module udp_command_parser #(
    parameter integer MAX_COMMANDS = 100
)(
    input  wire        clk,
    input  wire        rstn,
    input  wire        clear,
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
localparam logic [1:0] COMMAND_RESET = 2'd3;
localparam integer BUFFER_WIDTH = 46;
localparam integer BUFFER_ADDRESS_WIDTH =
    (MAX_COMMANDS <= 2) ? 1 : $clog2(MAX_COMMANDS);

logic        receiving;
logic [10:0] byte_index;
logic [7:0]  frame_header;
logic [7:0]  function_code;
logic [7:0]  data_count;
logic [2:0]  command_byte_index;
logic [7:0]  parsed_command_count;
logic        packet_invalid;
logic [7:0]  start_valve;
logic [7:0]  end_valve;
logic [15:0] parameter_0;
logic [15:0] parameter_1;
logic [31:0] received_crc;
logic [31:0] calculated_crc;
logic        pending_valid;
logic [8:0]  pending_count;

// A packet is kept private until both the application CRC and the Ethernet
// FCS have passed. The memory is then drained through the valid/ready port.
(* ramstyle = "M9K, no_rw_check" *) logic [BUFFER_WIDTH-1:0]
    command_buffer [0:MAX_COMMANDS-1];
logic [BUFFER_WIDTH-1:0] buffer_read_data;
logic [BUFFER_ADDRESS_WIDTH-1:0] buffer_read_address;
logic                             buffer_write_enable;
logic [BUFFER_ADDRESS_WIDTH-1:0] buffer_write_address;
logic [BUFFER_WIDTH-1:0]         buffer_write_data;
logic        draining;
logic [BUFFER_ADDRESS_WIDTH-1:0] emit_index;
logic [8:0]  emit_remaining;
logic [7:0]  emit_packet_count;

logic        crc_start;
logic        crc_enable;
logic        crc_end;
wire [31:0] crc32_value;
wire        crc32_valid;

wire [10:0] command_bytes =
    ({3'd0, data_count} << 2) + ({3'd0, data_count} << 1);
wire [10:0] command_data_end_index = command_bytes + 11'd2;
wire [10:0] command_payload_length = command_bytes + 11'd7;
wire        output_can_advance = !command_valid || command_ready;
wire        parser_can_start = !draining && !command_valid;
wire        storing_open_command =
    receiving && udp_rxdv && (function_code == 8'h01) &&
    (byte_index >= 3) && (byte_index <= command_data_end_index) &&
    (command_byte_index == 3'd5) &&
    (parsed_command_count < MAX_COMMANDS);
wire        accepting_set_packet =
    receiving && udp_rxdv && udp_rxend &&
    (frame_header == 8'hFF) && (function_code == 8'h02) &&
    (udp_rxamount == 16'd10) && (byte_index == 11'd9) &&
    (parameter_1 >= 16'd1) && (parameter_1 <= 16'd10) &&
    (calculated_crc == {received_crc[23:0], udp_rxdata});
wire        accepting_reset_packet =
    receiving && udp_rxdv && udp_rxend &&
    (frame_header == 8'hFF) && (function_code == 8'h03) &&
    (udp_rxamount == 16'd6) && (byte_index == 11'd5) &&
    (calculated_crc == {received_crc[23:0], udp_rxdata});

// Command record layout used by the command FIFO:
// [63:62] opcode, [61:56] start, [55:50] end,
// [49:34] parameter_0, [33:18] parameter_1,
// [17] final command in this packet, [16:9] packet command count,
// [8:0] reserved. The count lets the scheduler reserve a complete 0x01
// packet before the first record is appended.

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

always_comb begin
    buffer_write_enable  = 1'b0;
    buffer_write_address = '0;
    buffer_write_data    = '0;

    if (storing_open_command) begin
        buffer_write_enable = 1'b1;
        buffer_write_address =
            parsed_command_count[BUFFER_ADDRESS_WIDTH-1:0];
        buffer_write_data = {
            COMMAND_OPEN,
            start_valve[5:0],
            end_valve[5:0],
            parameter_0,
            parameter_1[15:8],
            udp_rxdata
        };
    end else if (accepting_set_packet) begin
        buffer_write_enable  = 1'b1;
        buffer_write_address = '0;
        buffer_write_data = {
            COMMAND_SET,
            6'd0,
            6'd0,
            parameter_0,
            parameter_1
        };
    end else if (accepting_reset_packet) begin
        buffer_write_enable  = 1'b1;
        buffer_write_address = '0;
        buffer_write_data = {
            COMMAND_RESET,
            6'd0,
            6'd0,
            16'd0,
            16'd0
        };
    end
end

// Prefetch the next command whenever the current output advances. Keeping
// all RAM access in this clocked block lets Quartus infer M9K storage instead
// of implementing the 100-command packet buffer as registers and a large mux.
always_comb begin
    buffer_read_address = emit_index;
    if (udp_rxframe_done && pending_valid)
        buffer_read_address = '0;
    else if (draining && output_can_advance && (emit_remaining > 1'b1))
        buffer_read_address = emit_index + 1'b1;
end

always_ff @(posedge clk) begin
    buffer_read_data <= command_buffer[buffer_read_address];
    if (buffer_write_enable)
        command_buffer[buffer_write_address] <= buffer_write_data;
end

// CRC covers 0xFF, function code, Data-Count and every command byte. The
// four CRC bytes in the UDP payload are transmitted most-significant byte
// first. Function 0x02 retains its existing six-byte pre-CRC layout, while
// function 0x03 ends its CRC-covered data after the function byte itself.
always_comb begin
    crc_start  = 1'b0;
    crc_enable = 1'b0;
    crc_end    = 1'b0;

    if (udp_rxdv) begin
        if ((udp_rxstart && parser_can_start) ||
            (receiving && byte_index == 0)) begin
            crc_start  = 1'b1;
            crc_enable = 1'b1;
        end else if (receiving) begin
            if (byte_index == 1) begin
                crc_enable = 1'b1;
                crc_end    = (udp_rxdata == 8'h03);
            end else if ((function_code == 8'h01) &&
                         (byte_index >= 2) &&
                         (byte_index <= command_data_end_index)) begin
                crc_enable = 1'b1;
                crc_end    = (data_count != 0) &&
                             (byte_index == command_data_end_index);
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
        receiving            <= 1'b0;
        byte_index           <= '0;
        frame_header         <= '0;
        function_code        <= '0;
        data_count           <= '0;
        command_byte_index   <= '0;
        parsed_command_count <= '0;
        packet_invalid       <= 1'b0;
        start_valve          <= '0;
        end_valve            <= '0;
        parameter_0          <= '0;
        parameter_1          <= '0;
        received_crc         <= '0;
        calculated_crc       <= '0;
        pending_valid        <= 1'b0;
        pending_count        <= '0;
        draining             <= 1'b0;
        emit_index           <= '0;
        emit_remaining       <= '0;
        emit_packet_count    <= '0;
        command_valid        <= 1'b0;
        command_data         <= '0;
    end else if (clear) begin
        receiving            <= 1'b0;
        byte_index           <= '0;
        frame_header         <= '0;
        function_code        <= '0;
        data_count           <= '0;
        command_byte_index   <= '0;
        parsed_command_count <= '0;
        packet_invalid       <= 1'b0;
        start_valve          <= '0;
        end_valve            <= '0;
        parameter_0          <= '0;
        parameter_1          <= '0;
        received_crc         <= '0;
        calculated_crc       <= '0;
        pending_valid        <= 1'b0;
        pending_count        <= '0;
        draining             <= 1'b0;
        emit_index           <= '0;
        emit_remaining       <= '0;
        emit_packet_count    <= '0;
        command_valid        <= 1'b0;
        command_data         <= '0;
    end else begin
        // Hold a command stable during backpressure. A synchronous buffer
        // read loads the next command as soon as the current one is accepted.
        if (command_valid && command_ready)
            command_valid <= 1'b0;

        if (draining && output_can_advance) begin
            command_data <= {
                buffer_read_data,
                (emit_remaining == 9'd1),
                emit_packet_count,
                9'd0
            };
            command_valid <= 1'b1;

            if (emit_remaining == 9'd1) begin
                draining       <= 1'b0;
                emit_remaining <= '0;
            end else begin
                emit_index     <= emit_index + 1'b1;
                emit_remaining <= emit_remaining - 1'b1;
            end
        end

        // A new frame also abandons a decoded packet whose Ethernet FCS did
        // not validate (such a frame never produces udp_rxframe_done).
        if (udp_rxstart && parser_can_start) begin
            receiving            <= 1'b1;
            byte_index           <= udp_rxdv ? 11'd1 : 11'd0;
            frame_header         <= udp_rxdv ? udp_rxdata : 8'd0;
            function_code        <= '0;
            data_count           <= '0;
            command_byte_index   <= '0;
            parsed_command_count <= '0;
            packet_invalid       <= 1'b0;
            start_valve          <= '0;
            end_valve            <= '0;
            parameter_0          <= '0;
            parameter_1          <= '0;
            received_crc         <= '0;
            calculated_crc       <= '0;
            pending_valid        <= 1'b0;
            pending_count        <= '0;
        end else if (receiving && udp_rxdv) begin
            byte_index <= byte_index + 1'b1;

            if (byte_index == 0) begin
                frame_header <= udp_rxdata;
            end else if (byte_index == 1) begin
                function_code <= udp_rxdata;
            end else if ((function_code == 8'h01) && (byte_index == 2)) begin
                data_count <= udp_rxdata;
                if ((udp_rxdata == 0) || (udp_rxdata > MAX_COMMANDS))
                    packet_invalid <= 1'b1;
            end else if ((function_code == 8'h01) &&
                         (byte_index >= 3) &&
                         (byte_index <= command_data_end_index)) begin
                case (command_byte_index)
                    3'd0: start_valve       <= udp_rxdata;
                    3'd1: end_valve         <= udp_rxdata;
                    3'd2: parameter_0[15:8] <= udp_rxdata;
                    3'd3: parameter_0[7:0]  <= udp_rxdata;
                    3'd4: parameter_1[15:8] <= udp_rxdata;
                    default: begin
                        parameter_1[7:0] <= udp_rxdata;
                        if ((start_valve > end_valve) ||
                            (end_valve > 8'd63) ||
                            ({parameter_1[15:8], udp_rxdata} == 16'd0)) begin
                            packet_invalid <= 1'b1;
                        end
                        parsed_command_count <= parsed_command_count + 1'b1;
                    end
                endcase

                if (command_byte_index == 3'd5)
                    command_byte_index <= 3'd0;
                else
                    command_byte_index <= command_byte_index + 1'b1;
            end else if (function_code == 8'h02) begin
                case (byte_index)
                    11'd2: parameter_0[15:8] <= udp_rxdata;
                    11'd3: parameter_0[7:0]  <= udp_rxdata;
                    11'd4: parameter_1[15:8] <= udp_rxdata;
                    11'd5: parameter_1[7:0]  <= udp_rxdata;
                    default: begin
                    end
                endcase
            end

            if (((function_code == 8'h01) &&
                 (byte_index > command_data_end_index) &&
                 (byte_index <= command_data_end_index + 11'd4)) ||
                ((function_code == 8'h02) &&
                 (byte_index >= 6) && (byte_index <= 9)) ||
                ((function_code == 8'h03) &&
                 (byte_index >= 2) && (byte_index <= 5))) begin
                received_crc <= {received_crc[23:0], udp_rxdata};
            end

            if (crc32_valid)
                calculated_crc <= crc32_value;

            if (udp_rxend) begin
                receiving <= 1'b0;

                if ((frame_header == 8'hFF) &&
                    (function_code == 8'h01) &&
                    !packet_invalid &&
                    (data_count >= 1) && (data_count <= MAX_COMMANDS) &&
                    (udp_rxamount == command_payload_length) &&
                    (byte_index == command_payload_length - 1'b1) &&
                    (parsed_command_count == data_count) &&
                    (calculated_crc == {received_crc[23:0], udp_rxdata})) begin
                    pending_valid <= 1'b1;
                    pending_count <= {1'b0, data_count};
                end else if ((frame_header == 8'hFF) &&
                             (function_code == 8'h02) &&
                             (udp_rxamount == 16'd10) &&
                             (byte_index == 11'd9) &&
                             (parameter_1 >= 16'd1) &&
                             (parameter_1 <= 16'd10) &&
                             (calculated_crc ==
                                {received_crc[23:0], udp_rxdata})) begin
                    pending_valid <= 1'b1;
                    pending_count <= 9'd1;
                end else if ((frame_header == 8'hFF) &&
                             (function_code == 8'h03) &&
                             (udp_rxamount == 16'd6) &&
                             (byte_index == 11'd5) &&
                             (calculated_crc ==
                                {received_crc[23:0], udp_rxdata})) begin
                    pending_valid <= 1'b1;
                    pending_count <= 9'd1;
                end else begin
                    pending_valid <= 1'b0;
                    pending_count <= '0;
                end
            end
        end

        // udp_rxframe_done is asserted only after the complete Ethernet FCS
        // has validated, so no command from a corrupt frame can escape.
        if (udp_rxframe_done && pending_valid) begin
            pending_valid  <= 1'b0;
            draining       <= 1'b1;
            emit_index     <= '0;
            emit_remaining <= pending_count;
            emit_packet_count <= pending_count[7:0];
        end
    end
end

endmodule
