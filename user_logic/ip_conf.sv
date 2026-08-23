module ip_conf #(
    parameter logic [47:0] BASE_BOARD_MAC_ADDR = 48'h60_A8_01_33_44_00,
    parameter logic [31:0] BASE_BOARD_IP_ADDR  = 32'hC0_A8_C8_32
)(
    input  wire         clk,
    input  wire         rstn,
    input  wire  [3:0]  addr,
    output logic [47:0] board_mac_addr,
    output logic [31:0] board_ip_addr
);

logic [3:0] addr_sync_1;
logic [3:0] addr_sync_2;
logic [1:0] sync_count;
logic       config_latched;

// addr comes from asynchronous DIP switches. After the top-level power-on
// reset is released, synchronize and latch the switch value once so that
// MAC/IP remain stable while the network stack is running.
always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        addr_sync_1    <= '0;
        addr_sync_2    <= '0;
        sync_count     <= '0;
        config_latched <= 1'b0;
        board_mac_addr <= BASE_BOARD_MAC_ADDR -1;
        board_ip_addr  <= BASE_BOARD_IP_ADDR  -1;
    end else begin
        addr_sync_1 <= addr;
        addr_sync_2 <= addr_sync_1;

        if (!config_latched) begin
            if (sync_count == 2) begin
                board_mac_addr <=
                    BASE_BOARD_MAC_ADDR + addr_sync_2;
                board_ip_addr <=
                    BASE_BOARD_IP_ADDR +  addr_sync_2;
                config_latched <= 1'b1;
            end else begin
                sync_count <= sync_count + 1'b1;
            end
        end
    end
end

endmodule
