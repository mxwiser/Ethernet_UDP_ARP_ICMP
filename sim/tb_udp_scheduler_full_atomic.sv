`timescale 1ns/1ps

module tb_udp_scheduler_full_atomic;

localparam integer SCHEDULE_DEPTH = 16;

logic clk = 1'b0;
logic rstn = 1'b0;
logic udp_rxstart = 1'b0;
logic udp_rxend = 1'b0;
logic udp_rxframe_done = 1'b0;
logic udp_rxdv = 1'b0;
logic [7:0] udp_rxdata = 8'd0;
logic [15:0] udp_rxamount = 16'd0;

wire scheduler_reset_pulse;
wire parser_valid;
wire [63:0] parser_data;
wire fifo_empty;
wire fifo_full;
wire [63:0] fifo_data;
wire fifo_read;
wire controller_ready;
wire [63:0] valve_open_status;
wire pwm_s1_wr_en;
wire [5:0] pwm_s1_wr_addr;
wire [3:0] pwm_s1_wr_duty;
wire pwm_s2_wr_en;
wire [5:0] pwm_s2_wr_addr;
wire [3:0] pwm_s2_wr_duty;

logic [7:0] payload [0:606];
integer payload_length;
integer byte_index;
integer command_index;
integer failures = 0;
logic reset_saw_parser_backlog = 1'b0;

always #5 clk = ~clk;

udp_scheduler_reset_detector u_reset_detector (
    .clk(clk), .rstn(rstn),
    .udp_rxstart(udp_rxstart), .udp_rxend(udp_rxend),
    .udp_rxframe_done(udp_rxframe_done), .udp_rxdv(udp_rxdv),
    .udp_rxdata(udp_rxdata), .udp_rxamount(udp_rxamount),
    .reset_pulse(scheduler_reset_pulse)
);

udp_command_parser #(.MAX_COMMANDS(100)) u_parser (
    .clk(clk), .rstn(rstn), .clear(scheduler_reset_pulse),
    .udp_rxstart(udp_rxstart), .udp_rxend(udp_rxend),
    .udp_rxframe_done(udp_rxframe_done), .udp_rxdv(udp_rxdv),
    .udp_rxdata(udp_rxdata), .udp_rxamount(udp_rxamount),
    .command_ready(!fifo_full), .command_valid(parser_valid),
    .command_data(parser_data)
);

command_fifo #(.DATA_WIDTH(64), .DEPTH(128)) u_fifo (
    .clk(clk), .rstn(rstn), .clear(scheduler_reset_pulse),
    .wr_en(parser_valid), .wr_data(parser_data),
    .rd_en(fifo_read), .rd_data(fifo_data),
    .empty(fifo_empty), .full(fifo_full)
);

assign fifo_read = !fifo_empty && controller_ready;

valve_controller #(
    .CLK_FREQ_HZ(1_000_000), .TIMER_HZ(1_000),
    .VALVE_COUNT(64), .PWM_LEVELS(10),
    .SCHEDULE_DEPTH(SCHEDULE_DEPTH)
) u_controller (
    .clk(clk), .rstn(rstn), .scheduler_reset(scheduler_reset_pulse),
    .command_valid(fifo_read), .command_data(fifo_data),
    .command_ready(controller_ready),
    .pwm_s1_wr_en(pwm_s1_wr_en), .pwm_s1_wr_addr(pwm_s1_wr_addr),
    .pwm_s1_wr_duty(pwm_s1_wr_duty),
    .pwm_s2_wr_en(pwm_s2_wr_en), .pwm_s2_wr_addr(pwm_s2_wr_addr),
    .pwm_s2_wr_duty(pwm_s2_wr_duty),
    .valve_open_status(valve_open_status)
);

function automatic [31:0] crc32_byte(
    input [31:0] crc_in,
    input [7:0] datum
);
    integer bit_number;
    reg [31:0] crc;
    begin
        crc = crc_in ^ datum;
        for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
            crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
        crc32_byte = crc;
    end
endfunction

task automatic append_crc(
    input integer data_length,
    input logic corrupt
);
    reg [31:0] crc;
    begin
        crc = 32'hffff_ffff;
        for (byte_index = 0; byte_index < data_length;
             byte_index = byte_index + 1)
            crc = crc32_byte(crc, payload[byte_index]);
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

task automatic build_open_packet(input integer command_count);
    integer base;
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h01;
        payload[2] = command_count;
        for (command_index = 0; command_index < command_count;
             command_index = command_index + 1) begin
            base = 3 + command_index * 6;
            payload[base+0] = command_index % 64;
            payload[base+1] = command_index % 64;
            payload[base+2] = 8'h03;
            payload[base+3] = 8'he8; // delay 1000 ms
            payload[base+4] = 8'h03;
            payload[base+5] = 8'he8; // duration 1000 ms
        end
        append_crc(3 + command_count * 6, 1'b0);
    end
endtask

task automatic build_reset_packet(input logic corrupt_crc);
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h03;
        append_crc(2, corrupt_crc);
    end
endtask

task automatic transmit_current_packet(
    input logic wait_for_parser,
    input logic send_fcs
);
    begin
        if (wait_for_parser)
            wait (!u_parser.receiving && !u_parser.draining && !parser_valid);

        udp_rxamount = payload_length;
        @(negedge clk);
        udp_rxstart = 1'b1;
        @(negedge clk);
        udp_rxstart = 1'b0;

        for (byte_index = 0; byte_index < payload_length;
             byte_index = byte_index + 1) begin
            udp_rxdv   = 1'b1;
            udp_rxdata = payload[byte_index];
            udp_rxend  = (byte_index == payload_length - 1);
            @(negedge clk);
        end
        udp_rxdv  = 1'b0;
        udp_rxend = 1'b0;

        repeat (3) @(negedge clk);
        if (send_fcs) begin
            udp_rxframe_done = 1'b1;
            @(negedge clk);
            udp_rxframe_done = 1'b0;
        end
    end
endtask

task automatic wait_pipeline_idle;
    begin
        wait (!u_parser.receiving && !u_parser.draining && !parser_valid &&
              fifo_empty && !u_controller.batch_active &&
              !u_controller.drop_batch &&
              !u_controller.schedule_refresh_pending &&
              (u_controller.fsm_state == 4'd1));
        repeat (2) @(posedge clk);
    end
endtask

always @(posedge clk) begin
    if (scheduler_reset_pulse && u_parser.draining &&
        (u_parser.emit_remaining != 0)) begin
        reset_saw_parser_backlog <= 1'b1;
    end
end

initial begin
    #5_000_000;
    $fatal(1, "FULL_ATOMIC_TIMEOUT count=%0d fifo=%0d parser_remaining=%0d",
           u_controller.schedule_active_count, u_fifo.item_count,
           u_parser.emit_remaining);
end

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (controller_ready);

    // Leave exactly two slots free.
    build_open_packet(14);
    transmit_current_packet(1'b1, 1'b1);
    wait_pipeline_idle();
    if (u_controller.schedule_active_count != 14) begin
        $display("FULL_ATOMIC_INITIAL_FILL_FAIL count=%0d",
                 u_controller.schedule_active_count);
        failures = failures + 1;
    end

    // Three commands cannot fit: reject all three, not just the third.
    build_open_packet(3);
    transmit_current_packet(1'b1, 1'b1);
    wait_pipeline_idle();
    if ((u_controller.schedule_active_count != 14) ||
        (u_controller.rejected_packet_count != 1)) begin
        $display("FULL_ATOMIC_REJECT_FAIL count=%0d rejects=%0d",
                 u_controller.schedule_active_count,
                 u_controller.rejected_packet_count);
        failures = failures + 1;
    end

    // A packet exactly matching the two free slots is accepted in full.
    build_open_packet(2);
    transmit_current_packet(1'b1, 1'b1);
    wait_pipeline_idle();
    if ((u_controller.schedule_active_count != SCHEDULE_DEPTH) ||
        !u_controller.schedule_full) begin
        $display("FULL_ATOMIC_EXACT_FIT_FAIL count=%0d full=%0b",
                 u_controller.schedule_active_count,
                 u_controller.schedule_full);
        failures = failures + 1;
    end

    // Neither a bad application CRC nor a missing Ethernet FCS may reset it.
    build_reset_packet(1'b1);
    transmit_current_packet(1'b1, 1'b1);
    repeat (20) @(posedge clk);
    if (u_controller.schedule_active_count != SCHEDULE_DEPTH) begin
        $display("FULL_ATOMIC_BAD_CRC_RESET_FAIL count=%0d",
                 u_controller.schedule_active_count);
        failures = failures + 1;
    end

    build_reset_packet(1'b0);
    transmit_current_packet(1'b1, 1'b0);
    repeat (20) @(posedge clk);
    if (u_controller.schedule_active_count != SCHEDULE_DEPTH) begin
        $display("FULL_ATOMIC_BAD_FCS_RESET_FAIL count=%0d",
                 u_controller.schedule_active_count);
        failures = failures + 1;
    end

    // While the full table causes this 100-command packet to be rejected and
    // the parser is still draining it, inject reset without waiting for the
    // normal parser. The independent detector must preempt and flush it all.
    build_open_packet(100);
    transmit_current_packet(1'b1, 1'b1);
    wait (u_parser.draining && (u_parser.emit_remaining > 90));
    build_reset_packet(1'b0);
    transmit_current_packet(1'b0, 1'b1);
    wait (scheduler_reset_pulse);
    @(posedge clk);
    #1;
    if ((u_controller.schedule_active_count != 0) || !fifo_empty ||
        u_parser.draining || parser_valid || u_controller.batch_active ||
        u_controller.drop_batch || !reset_saw_parser_backlog) begin
        $display("FULL_ATOMIC_RESET_BYPASS_FAIL count=%0d fifo_empty=%0b drain=%0b valid=%0b backlog=%0b",
                 u_controller.schedule_active_count, fifo_empty,
                 u_parser.draining, parser_valid, reset_saw_parser_backlog);
        failures = failures + 1;
    end

    // All addresses are reusable immediately after reset.
    wait_pipeline_idle();
    build_open_packet(16);
    transmit_current_packet(1'b1, 1'b1);
    wait_pipeline_idle();
    if (u_controller.schedule_active_count != SCHEDULE_DEPTH) begin
        $display("FULL_ATOMIC_POST_RESET_REFILL_FAIL count=%0d",
                 u_controller.schedule_active_count);
        failures = failures + 1;
    end

    if (failures == 0)
        $display("UDP_SCHEDULER_FULL_ATOMIC_PASS reject=whole exact_fit=16 reset_bypass=backlogged refill=16");
    else
        $fatal(1, "UDP_SCHEDULER_FULL_ATOMIC_FAIL failures=%0d", failures);
    $finish;
end

endmodule
