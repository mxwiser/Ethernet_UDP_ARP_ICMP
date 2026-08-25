`timescale 1ns/1ps

module tb_udp_command_parser;

logic        clk = 1'b0;
logic        rstn = 1'b0;
logic        udp_rxstart = 1'b0;
logic        udp_rxend = 1'b0;
logic        udp_rxframe_done = 1'b0;
logic        udp_rxdv = 1'b0;
logic [7:0]  udp_rxdata = 8'd0;
logic [15:0] udp_rxamount = 16'd0;
logic        command_ready = 1'b1;
wire         command_valid;
wire [63:0]  command_data;

logic [7:0]  payload [0:1023];
logic [63:0] accepted [0:255];
integer payload_length;
integer accepted_count = 0;
integer failures = 0;
integer i;

always #5 clk = ~clk;

initial begin
    #500000;
    $fatal(1,
        "PARSER_TIMEOUT accepted=%0d receiving=%0b pending=%0b draining=%0b valid=%0b index=%0d remaining=%0d",
        accepted_count, dut.receiving, dut.pending_valid, dut.draining,
        command_valid, dut.emit_index, dut.emit_remaining);
end

udp_command_parser #(
    .MAX_COMMANDS (100)
) dut (
    .clk              (clk),
    .rstn             (rstn),
    .clear            (1'b0),
    .udp_rxstart      (udp_rxstart),
    .udp_rxend        (udp_rxend),
    .udp_rxframe_done (udp_rxframe_done),
    .udp_rxdv         (udp_rxdv),
    .udp_rxdata       (udp_rxdata),
    .udp_rxamount     (udp_rxamount),
    .command_ready    (command_ready),
    .command_valid    (command_valid),
    .command_data     (command_data)
);

always @(posedge clk) begin
    if (rstn && command_valid && command_ready) begin
        accepted[accepted_count] = command_data;
        accepted_count = accepted_count + 1;
    end
end

function automatic [31:0] crc32_byte(
    input [31:0] crc_in,
    input [7:0] datum
);
    integer bit_index;
    reg [31:0] crc;
    begin
        crc = crc_in ^ datum;
        for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
            crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
        crc32_byte = crc;
    end
endfunction

task automatic append_crc(input integer data_length, input integer corrupt);
    integer index;
    reg [31:0] crc;
    begin
        crc = 32'hffff_ffff;
        for (index = 0; index < data_length; index = index + 1)
            crc = crc32_byte(crc, payload[index]);
        crc = crc ^ 32'hffff_ffff;
        if (corrupt)
            crc = crc ^ 32'h0000_0001;
        payload[data_length+0] = crc[31:24];
        payload[data_length+1] = crc[23:16];
        payload[data_length+2] = crc[15:8];
        payload[data_length+3] = crc[7:0];
        payload_length = data_length + 4;
    end
endtask

task automatic put_open_command(
    input integer command_index,
    input [7:0] start_valve,
    input [7:0] end_valve,
    input [15:0] delay_ms,
    input [15:0] duration_ms
);
    integer base;
    begin
        base = 3 + command_index * 6;
        payload[base+0] = start_valve;
        payload[base+1] = end_valve;
        payload[base+2] = delay_ms[15:8];
        payload[base+3] = delay_ms[7:0];
        payload[base+4] = duration_ms[15:8];
        payload[base+5] = duration_ms[7:0];
    end
endtask

task automatic begin_open_packet(input [7:0] count);
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h01;
        payload[2] = count;
    end
endtask

task automatic transmit_packet;
    integer index;
    begin
        udp_rxamount = payload_length;
        @(negedge clk);
        udp_rxstart = 1'b1;
        @(negedge clk);
        udp_rxstart = 1'b0;

        for (index = 0; index < payload_length; index = index + 1) begin
            udp_rxdv   = 1'b1;
            udp_rxdata = payload[index];
            udp_rxend  = (index == payload_length - 1);
            @(negedge clk);
        end

        udp_rxdv   = 1'b0;
        udp_rxend  = 1'b0;
        repeat (3) @(negedge clk);
        udp_rxframe_done = 1'b1;
        @(negedge clk);
        udp_rxframe_done = 1'b0;
    end
endtask

task automatic transmit_packet_without_fcs;
    integer index;
    begin
        udp_rxamount = payload_length;
        @(negedge clk);
        udp_rxstart = 1'b1;
        @(negedge clk);
        udp_rxstart = 1'b0;

        for (index = 0; index < payload_length; index = index + 1) begin
            udp_rxdv   = 1'b1;
            udp_rxdata = payload[index];
            udp_rxend  = (index == payload_length - 1);
            @(negedge clk);
        end

        udp_rxdv  = 1'b0;
        udp_rxend = 1'b0;
        repeat (3) @(negedge clk);
    end
endtask

task automatic expect_no_new_commands(
    input integer previous_count,
    input [127:0] case_name
);
    begin
        repeat (20) @(posedge clk);
        if (accepted_count != previous_count) begin
            $display("PARSER_REJECT_FAIL case=%0s before=%0d after=%0d",
                     case_name, previous_count, accepted_count);
            failures = failures + 1;
        end
    end
endtask

integer before_count;
logic [63:0] expected;
initial begin
    repeat (5) @(posedge clk);
    rstn = 1'b1;
    repeat (3) @(posedge clk);

    // The example from the protocol: all three records must survive as one
    // ordered batch, and output must remain stable under backpressure.
    begin_open_packet(8'd3);
    put_open_command(0, 8'd3, 8'd3, 16'd5000, 16'd5000);
    put_open_command(1, 8'd3, 8'd3, 16'd3000, 16'd5000);
    put_open_command(2, 8'd3, 8'd3, 16'd6000, 16'd2000);
    append_crc(21, 0);
    command_ready = 1'b0;
    transmit_packet();
    wait (command_valid);
    repeat (4) @(posedge clk);
    if (accepted_count != 0) begin
        $display("PARSER_BACKPRESSURE_FAIL accepted=%0d", accepted_count);
        failures = failures + 1;
    end
    @(negedge clk);
    command_ready = 1'b1;
    wait (accepted_count == 3);
    repeat (2) @(posedge clk);

    expected = {2'd1, 6'd3, 6'd3, 16'd5000, 16'd5000,
                1'b0, 8'd3, 9'd0};
    if (accepted[0] !== expected) begin
        $display("PARSER_DATA_FAIL index=0 got=%016x expected=%016x",
                 accepted[0], expected);
        failures = failures + 1;
    end
    expected = {2'd1, 6'd3, 6'd3, 16'd3000, 16'd5000,
                1'b0, 8'd3, 9'd0};
    if (accepted[1] !== expected) begin
        $display("PARSER_DATA_FAIL index=1 got=%016x expected=%016x",
                 accepted[1], expected);
        failures = failures + 1;
    end
    expected = {2'd1, 6'd3, 6'd3, 16'd6000, 16'd2000,
                1'b1, 8'd3, 9'd0};
    if (accepted[2] !== expected) begin
        $display("PARSER_DATA_FAIL index=2 got=%016x expected=%016x",
                 accepted[2], expected);
        failures = failures + 1;
    end

    // A bad application CRC rejects the whole packet.
    repeat (3) @(posedge clk);
    before_count = accepted_count;
    begin_open_packet(8'd2);
    put_open_command(0, 8'd1, 8'd1, 16'd1, 16'd2);
    put_open_command(1, 8'd2, 8'd2, 16'd3, 16'd4);
    append_crc(15, 1);
    transmit_packet();
    expect_no_new_commands(before_count, "bad_crc");

    // One bad range invalidates the complete batch.
    begin_open_packet(8'd2);
    put_open_command(0, 8'd1, 8'd1, 16'd1, 16'd2);
    put_open_command(1, 8'd9, 8'd8, 16'd3, 16'd4);
    append_crc(15, 0);
    transmit_packet();
    expect_no_new_commands(before_count, "bad_subcommand");

    // Existing function 0x02 remains a single six-byte data record.
    payload[0] = 8'hff;
    payload[1] = 8'h02;
    payload[2] = 8'h00;
    payload[3] = 8'h0f;
    payload[4] = 8'h00;
    payload[5] = 8'h05;
    append_crc(6, 0);
    transmit_packet();
    wait (accepted_count == before_count + 1);
    expected = {2'd2, 6'd0, 6'd0, 16'd15, 16'd5,
                1'b1, 8'd1, 9'd0};
    if (accepted[before_count] !== expected) begin
        $display("PARSER_FUNCTION_02_FAIL got=%016x expected=%016x",
                 accepted[before_count], expected);
        failures = failures + 1;
    end
    before_count = accepted_count;

    // Function 0x03 is exactly FF 03 CRC32 and emits one reset command.
    payload[0] = 8'hff;
    payload[1] = 8'h03;
    append_crc(2, 0);
    transmit_packet();
    wait (accepted_count == before_count + 1);
    expected = {2'd3, 6'd0, 6'd0, 16'd0, 16'd0,
                1'b1, 8'd1, 9'd0};
    if (accepted[before_count] !== expected) begin
        $display("PARSER_FUNCTION_03_FAIL got=%016x expected=%016x",
                 accepted[before_count], expected);
        failures = failures + 1;
    end
    before_count = accepted_count;

    // A bad reset CRC must not escape the parser.
    payload[0] = 8'hff;
    payload[1] = 8'h03;
    append_crc(2, 1);
    transmit_packet();
    expect_no_new_commands(before_count, "reset_bad_crc");

    // Extra payload data makes the reset packet length invalid.
    payload[0] = 8'hff;
    payload[1] = 8'h03;
    append_crc(2, 0);
    payload[6] = 8'h00;
    payload_length = 7;
    transmit_packet();
    expect_no_new_commands(before_count, "reset_bad_length");

    // Application CRC alone is insufficient; Ethernet FCS must also pass.
    payload[0] = 8'hff;
    payload[1] = 8'h03;
    append_crc(2, 0);
    transmit_packet_without_fcs();
    expect_no_new_commands(before_count, "reset_bad_fcs");

    // Exercise the exact 100-command upper bound.
    begin_open_packet(8'd100);
    for (i = 0; i < 100; i = i + 1)
        put_open_command(i, i % 64, i % 64, i, i + 1);
    append_crc(603, 0);
    transmit_packet();
    wait (accepted_count == before_count + 100);
    repeat (2) @(posedge clk);
    for (i = 0; i < 100; i = i + 1) begin
        expected = {
            2'd1,
            6'(i % 64),
            6'(i % 64),
            16'(i),
            16'(i + 1),
            (i == 99),
            8'd100,
            9'd0
        };
        if (accepted[before_count+i] !== expected) begin
            $display("PARSER_100_DATA_FAIL index=%0d got=%016x expected=%016x",
                     i, accepted[before_count+i], expected);
            failures = failures + 1;
        end
    end
    before_count = accepted_count;

    // Count 101 is structurally well formed but above the configured limit.
    begin_open_packet(8'd101);
    for (i = 0; i < 101; i = i + 1)
        put_open_command(i, i % 64, i % 64, i, i + 1);
    append_crc(609, 0);
    transmit_packet();
    expect_no_new_commands(before_count, "count_101");

    // Count zero is also invalid.
    begin_open_packet(8'd0);
    append_crc(3, 0);
    transmit_packet();
    expect_no_new_commands(before_count, "count_0");

    if (failures == 0)
        $display("UDP_COMMAND_PARSER_PASS commands=3+100 function02=pass function03=pass rejects=7 backpressure=pass");
    else
        $fatal(1, "UDP_COMMAND_PARSER_FAIL failures=%0d", failures);
    $finish;
end

endmodule
