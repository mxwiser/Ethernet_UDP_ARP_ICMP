module udp_command_parser(
    input  wire        clk,
    input  wire        rstn,
    input  wire        udp_rxstart,
    input  wire        udp_rxend,
    input  wire        udp_rxframe_done,
    input  wire        udp_rxdv,
    input  wire [7:0]  udp_rxdata,
    input  wire [15:0] udp_rxamount,

    input  wire        command_ready,
    output logic       command_valid,
    output logic [63:0] command_data
);

localparam logic [1:0] COMMAND_OPEN = 2'd1;
localparam logic [1:0] COMMAND_SET  = 2'd2;

logic        receiving;
logic [4:0]  byte_index;
logic [7:0]  frame_header;
logic [7:0]  function_code;
logic [7:0]  start_valve;
logic [7:0]  end_valve;
logic [15:0] parameter_0;
logic [15:0] parameter_1;
logic [31:0] received_crc;
logic [31:0] calculated_crc;
logic        pending_valid;
logic [63:0] pending_command;

logic        crc_start;
logic        crc_enable;
logic        crc_end;
wire [31:0] crc32_value;
wire        crc32_valid;

// Command record layout used by the command FIFO:
// [63:62] opcode, [61:56] start, [55:50] end,
// [49:34] parameter_0, [33:18] parameter_1, [17:0] reserved.

CRC32_D8 u_command_crc32 (
    .sys_clk     (clk),
    .sys_rst_n   (rstn),
    .data        (udp_rxdata),
    .crc_start   (crc_start),
    .crc_en      (crc_enable),
    .crc_end     (crc_end),
    .crc32       (crc32_value),
    .crc32_valid (crc32_valid)
);

// CRC covers 0xFF, function code and all non-CRC data bytes. The four CRC
// bytes in the UDP payload are transmitted most-significant byte first.
always_comb begin
    crc_start  = 1'b0;
    crc_enable = 1'b0;
    crc_end    = 1'b0;

    if (udp_rxdv) begin
        if (udp_rxstart || (receiving && byte_index == 0)) begin
            crc_start  = 1'b1;
            crc_enable = 1'b1;
        end else if (receiving) begin
            if (byte_index == 1) begin
                crc_enable = 1'b1;
            end else if ((function_code == 8'h01) &&
                         (byte_index >= 2) && (byte_index <= 7)) begin
                crc_enable = 1'b1;
                crc_end    = (byte_index == 7);
            end else if ((function_code == 8'h02) &&
                         (byte_index >= 2) && (byte_index <= 5)) begin
                crc_enable = 1'b1;
                crc_end    = (byte_index == 5);
            end
        end
    end
end

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        receiving       <= 1'b0;
        byte_index      <= '0;
        frame_header    <= '0;
        function_code   <= '0;
        start_valve     <= '0;
        end_valve       <= '0;
        parameter_0     <= '0;
        parameter_1     <= '0;
        received_crc    <= '0;
        calculated_crc  <= '0;
        pending_valid   <= 1'b0;
        pending_command <= '0;
        command_valid   <= 1'b0;
        command_data    <= '0;
    end else begin
        command_valid <= 1'b0;

        if (udp_rxstart) begin
            receiving      <= 1'b1;
            byte_index     <= udp_rxdv ? 5'd1 : 5'd0;
            frame_header   <= udp_rxdv ? udp_rxdata : 8'd0;
            function_code  <= '0;
            start_valve    <= '0;
            end_valve      <= '0;
            parameter_0    <= '0;
            parameter_1    <= '0;
            received_crc   <= '0;
            calculated_crc <= '0;
            pending_valid  <= 1'b0;
        end else if (receiving && udp_rxdv) begin
            byte_index <= byte_index + 1'b1;

            case (byte_index)
                5'd0: frame_header <= udp_rxdata;
                5'd1: function_code <= udp_rxdata;

                5'd2: begin
                    if (function_code == 8'h01)
                        start_valve <= udp_rxdata;
                    else if (function_code == 8'h02)
                        parameter_0[15:8] <= udp_rxdata;
                end

                5'd3: begin
                    if (function_code == 8'h01)
                        end_valve <= udp_rxdata;
                    else if (function_code == 8'h02)
                        parameter_0[7:0] <= udp_rxdata;
                end

                5'd4: begin
                    if (function_code == 8'h01)
                        parameter_0[15:8] <= udp_rxdata;
                    else if (function_code == 8'h02)
                        parameter_1[15:8] <= udp_rxdata;
                end

                5'd5: begin
                    if (function_code == 8'h01)
                        parameter_0[7:0] <= udp_rxdata;
                    else if (function_code == 8'h02)
                        parameter_1[7:0] <= udp_rxdata;
                end

                5'd6: begin
                    if (function_code == 8'h01)
                        parameter_1[15:8] <= udp_rxdata;
                end

                5'd7: begin
                    if (function_code == 8'h01)
                        parameter_1[7:0] <= udp_rxdata;
                end

                default: begin
                end
            endcase

            if (((function_code == 8'h01) && (byte_index >= 8)) ||
                ((function_code == 8'h02) && (byte_index >= 6))) begin
                received_crc <= {received_crc[23:0], udp_rxdata};
            end

            if (crc32_valid)
                calculated_crc <= crc32_value;

            if (udp_rxend) begin
                receiving <= 1'b0;

                if ((frame_header == 8'hFF) &&
                    (function_code == 8'h01) &&
                    (udp_rxamount == 16'd12) &&
                    (byte_index == 5'd11) &&
                    (start_valve <= end_valve) &&
                    (end_valve <= 8'd63) &&
                    (parameter_1 != 16'd0) &&
                    (calculated_crc == {received_crc[23:0], udp_rxdata})) begin
                    pending_command <= {
                        COMMAND_OPEN,
                        start_valve[5:0],
                        end_valve[5:0],
                        parameter_0,
                        parameter_1,
                        18'd0
                    };
                    pending_valid <= 1'b1;
                end else if ((frame_header == 8'hFF) &&
                             (function_code == 8'h02) &&
                             (udp_rxamount == 16'd10) &&
                             (byte_index == 5'd9) &&
                             (parameter_1 >= 16'd1) &&
                             (parameter_1 <= 16'd10) &&
                             (calculated_crc ==
                                {received_crc[23:0], udp_rxdata})) begin
                    pending_command <= {
                        COMMAND_SET,
                        6'd0,
                        6'd0,
                        parameter_0,
                        parameter_1,
                        18'd0
                    };
                    pending_valid <= 1'b1;
                end else begin
                    pending_valid <= 1'b0;
                end
            end
        end

        // Wait for the Ethernet frame (including padding/FCS) to finish before
        // committing the decoded command. A full FIFO silently drops it.
        if (udp_rxframe_done && pending_valid) begin
            if (command_ready) begin
                command_data  <= pending_command;
                command_valid <= 1'b1;
            end
            pending_valid <= 1'b0;
        end
    end
end

endmodule
