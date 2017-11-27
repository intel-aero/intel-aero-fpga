module i2c_slave(
    clk,
    master_clk,
    master_sda,
    slave_addr,
    slave_asserted,
    slave_in_tx_mode,
    slave_tx_buffer,
    slave_tx_request,
    slave_rx_buffer,
    slave_rx_available
);

input wire clk;
input wire master_clk;

inout wire master_sda;

input wire [0:6] slave_addr;

output reg slave_asserted = 0;
output reg slave_in_tx_mode = 0;

input wire [0:7] slave_tx_buffer;
output reg slave_tx_request = 0;

output reg [0:7] slave_rx_buffer;
output reg slave_rx_available = 0;

// internal data
reg [1:0] master_clk_edge;
reg [1:0] master_sda_edge;
reg master_clk_rising_edge = 0;
reg master_clk_falling_edge = 0;

reg i2c_start = 0;
reg i2c_stop = 0;

parameter state_idle = 4'd0;
parameter state_start_issued_waiting_slave_addr = 4'd1;
parameter state_slave_asserted_need_to_send_ack = 4'd2;
parameter state_slave_asserted_in_master_write_mode = 4'd3;
parameter state_slave_asserted_in_master_read_mode = 4'd4;
parameter state_slave_asserted_in_master_read_mode_waiting_ack_assert_highz = 4'd5;
parameter state_slave_asserted_in_master_read_mode_waiting_ack_read = 4'd6;
parameter state_slave_asserted_in_master_write_mode_assert_ack = 4'd7;
parameter state_slave_asserted_in_master_write_mode_assert_highz_ack = 4'd8;
reg [3:0] state = state_idle;

reg [2:0] count = 0;
reg sda_write = 1'bz;

// assigns
assign master_sda = sda_write;

always @(posedge clk) begin
    // helper to detect edges
    master_clk_edge <= { master_clk_edge[0], master_clk };
    master_sda_edge <= { master_sda_edge[0], master_sda };
    if (master_clk_edge[0] && !master_clk_edge[1]) begin
        master_clk_rising_edge <= 1;
    end else begin
        master_clk_rising_edge <= 0;
    end
    if (master_clk_edge[1] && !master_clk_edge[0]) begin
        master_clk_falling_edge <= 1;
    end else begin
        master_clk_falling_edge <= 0;
    end

    i2c_start <= master_sda_edge[1] && !master_sda_edge[0] && master_clk_edge[0];
    i2c_stop <= master_sda_edge[0] && !master_sda_edge[1] && master_clk_edge[0];
end

always @(posedge clk) begin
    if (i2c_start || i2c_stop) begin
        state <= i2c_start ? state_start_issued_waiting_slave_addr : state_idle;
        count <= 3'd0;
        sda_write <= 1'bz;
        slave_asserted <= 1'd0;
        slave_tx_request <= 1'd0;
        slave_rx_available <= 1'd0;
    end else begin
        if (master_clk_rising_edge) begin
            case (state)
            state_start_issued_waiting_slave_addr: begin
                slave_rx_buffer[count] <= master_sda;
                count <= count + 3'd1;
                // last data?
                if (count == 3'd7) begin
                    // is this slave addr?
                    if (slave_rx_buffer[0:6] == slave_addr) begin
                        state <= state_slave_asserted_need_to_send_ack;
                        slave_asserted <= 1'd1;
                    end else begin
                        state <= state_idle;
                    end
                end
            end
            state_slave_asserted_in_master_read_mode_waiting_ack_read: begin
                if (master_sda) begin
                    // got a nack
                    state <= state_idle;
                end else begin
                    state <= state_slave_asserted_in_master_read_mode;
                end
            end
            state_slave_asserted_in_master_write_mode: begin
                slave_rx_buffer[count] <= master_sda;
                count <= count + 3'd1;
                // last data?
                if (count == 3'd7) begin
                    state <= state_slave_asserted_in_master_write_mode_assert_ack;
                    slave_rx_available <= 1'd1;
                end else begin
                    slave_rx_available <= 1'd0;
                end
            end
            endcase
        end else if (master_clk_falling_edge) begin
            case (state)
            state_slave_asserted_need_to_send_ack: begin
                sda_write <= 1'd0;
                state <= slave_rx_buffer[7] ? state_slave_asserted_in_master_read_mode : state_slave_asserted_in_master_write_mode_assert_highz_ack;
                slave_in_tx_mode <= slave_rx_buffer[7];
                slave_tx_request <= 1'd1;
            end
            state_slave_asserted_in_master_read_mode: begin
                sda_write <= (slave_tx_buffer[count] ? 1'bz : 1'd0);
                count <= count + 3'd1;
                // last data?
                if (count == 3'd7) begin
                    state <= state_slave_asserted_in_master_read_mode_waiting_ack_assert_highz;
                    slave_tx_request <= 1'd1;
                end else begin
                    slave_tx_request <= 1'd0;
                end
            end
            state_slave_asserted_in_master_read_mode_waiting_ack_assert_highz: begin
                sda_write <= 1'bz;
                state <= state_slave_asserted_in_master_read_mode_waiting_ack_read;
            end
            state_slave_asserted_in_master_write_mode: begin
                sda_write <= 1'bz;
            end
            state_slave_asserted_in_master_write_mode_assert_ack: begin
                sda_write <= 1'd0;
                state <= state_slave_asserted_in_master_write_mode_assert_highz_ack;
            end
            state_slave_asserted_in_master_write_mode_assert_highz_ack: begin
                sda_write <= 1'bz;
                state <= state_slave_asserted_in_master_write_mode;
            end
            endcase
        end
    end
end

endmodule