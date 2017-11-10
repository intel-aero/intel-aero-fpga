module i2c_bridge_new(
        clk,
        master_clk,
        master_sda,
        slave_clk,
        slave_sda
);

input wire clk;

input wire master_clk;
inout wire master_sda;

output wire slave_clk;
inout wire slave_sda;

// internal data
reg [1:0] master_clk_edge;
reg [1:0] master_sda_edge;
reg master_clk_rising_edge = 1'd0;
reg master_clk_falling_edge = 1'd0;

reg i2c_start = 1'd0;
reg i2c_stop = 1'd0;

parameter state_idle = 3'd0;
parameter state_waiting_slave_addr = 3'd1;
parameter state_let_slave_ack_or_nack = 3'd2;
parameter state_read_slave_ack_or_nack = 3'd3;
parameter state_data_transfer = 3'd4;
parameter state_data_waiting_ack = 3'd5;
parameter state_data_begin_transfer = 3'd6;
reg [2:0] state = state_idle;

reg [2:0] count = 3'd0;
reg master_wants_to_read = 1'd0;
reg slave_write = 1'd0;

assign slave_clk = master_clk;

assign slave_sda = (!slave_write) ? (master_sda ? 1'bz : 1'd0) : 1'bz;
assign master_sda = (slave_write) ? (slave_sda ? 1'bz : 1'd0) : 1'bz;

always @(posedge clk) begin
    // helper to detect edges
    master_clk_edge <= { master_clk_edge[0], master_clk };
    master_sda_edge <= { master_sda_edge[0], master_sda };
    if (master_clk_edge[0] && !master_clk_edge[1]) begin
        master_clk_rising_edge <= 1'd1;
    end else begin
        master_clk_rising_edge <= 1'd0;
    end
    if (master_clk_edge[1] && !master_clk_edge[0]) begin
        master_clk_falling_edge <= 1'd1;
    end else begin
        master_clk_falling_edge <= 1'd0;
    end

    i2c_start <= master_sda_edge[1] && !master_sda_edge[0] && master_clk_edge[0];
    i2c_stop <= master_sda_edge[0] && !master_sda_edge[1] && master_clk_edge[0];
end

always @(posedge clk) begin
    if (i2c_start || i2c_stop) begin
        state <= i2c_start ? state_waiting_slave_addr : state_idle;
        count <= 3'd0;
        slave_write <= 1'd0;
        master_wants_to_read <= 1'd0;
    end else begin
        if (master_clk_rising_edge) begin
            case (state)
            state_waiting_slave_addr: begin
                count <= count + 3'd1;
                if (count == 3'd7) begin
                    state <= state_let_slave_ack_or_nack;
                    master_wants_to_read <= master_sda;
                end
            end
            state_read_slave_ack_or_nack: begin
                if (slave_sda) begin
                    // nack
                    slave_write <= 1'd0;
                    state <= state_idle;
                end else begin
                    // ack
                    state <= state_data_begin_transfer;
                end
			end
            endcase
        end else if (master_clk_falling_edge) begin
            case (state)
            state_let_slave_ack_or_nack: begin
                slave_write <= 1'd1;
                state <= state_read_slave_ack_or_nack;
            end
            state_data_begin_transfer: begin
                slave_write <= master_wants_to_read;
                state <= state_data_transfer;
            end
            state_data_transfer: begin
                count <= count + 3'd1;
                if (count == 3'd7) begin
                    slave_write <= !master_wants_to_read;
                    state <= state_data_waiting_ack;
                end
            end
            state_data_waiting_ack: begin
                slave_write <= master_wants_to_read;
                state <= state_data_transfer;
            end
            endcase
        end
    end
end

endmodule