`timescale 1ns/1ps

module tb_udp_scheduler_reset;

logic clk = 1'b0;
logic rstn = 1'b0;
logic udp_rxstart = 1'b0;
logic udp_rxend = 1'b0;
logic udp_rxframe_done = 1'b0;
logic udp_rxdv = 1'b0;
logic [7:0] udp_rxdata = 8'd0;
logic [15:0] udp_rxamount = 16'd0;

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

logic [7:0] payload [0:12];
integer payload_length;
integer byte_index;
integer failures = 0;
integer reset_accept_count = 0;
integer reset_accept_tick = -1;
integer valve4_open_tick = -1;
integer valve4_close_tick = -1;
logic valve4_seen_open = 1'b0;
logic reset_completed = 1'b0;

always #5 clk = ~clk;

udp_command_parser #(.MAX_COMMANDS(100)) u_parser (
    .clk(clk), .rstn(rstn), .clear(1'b0),
    .udp_rxstart(udp_rxstart), .udp_rxend(udp_rxend),
    .udp_rxframe_done(udp_rxframe_done), .udp_rxdv(udp_rxdv),
    .udp_rxdata(udp_rxdata), .udp_rxamount(udp_rxamount),
    .command_ready(!fifo_full), .command_valid(parser_valid),
    .command_data(parser_data)
);

command_fifo #(.DATA_WIDTH(64), .DEPTH(128)) u_fifo (
    .clk(clk), .rstn(rstn), .clear(1'b0), .wr_en(parser_valid),
    .wr_data(parser_data), .rd_en(fifo_read), .rd_data(fifo_data),
    .empty(fifo_empty), .full(fifo_full)
);

assign fifo_read = !fifo_empty && controller_ready;

valve_controller #(
    .CLK_FREQ_HZ(500_000), .TIMER_HZ(100),
    .VALVE_COUNT(64), .PWM_LEVELS(10), .SCHEDULE_DEPTH(1024)
) u_controller (
    .clk(clk), .rstn(rstn), .scheduler_reset(1'b0),
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

task automatic transmit_packet;
    begin
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
        udp_rxframe_done = 1'b1;
        @(negedge clk);
        udp_rxframe_done = 1'b0;
    end
endtask

task automatic send_udp_open(
    input [7:0] first_valve,
    input [7:0] last_valve,
    input [15:0] delay_ms,
    input [15:0] duration_ms
);
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h01;
        payload[2] = 8'd1;
        payload[3] = first_valve;
        payload[4] = last_valve;
        payload[5] = delay_ms[15:8];
        payload[6] = delay_ms[7:0];
        payload[7] = duration_ms[15:8];
        payload[8] = duration_ms[7:0];
        append_crc(9, 1'b0);
        transmit_packet();
    end
endtask

task automatic send_udp_reset(input logic corrupt_crc);
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h03;
        append_crc(2, corrupt_crc);
        transmit_packet();
    end
endtask

always @(posedge clk) begin
    if (rstn && fifo_read && (fifo_data[63:62] == 2'd3)) begin
        reset_accept_tick = u_controller.current_time;
        reset_accept_count = reset_accept_count + 1;
    end
end

always @(posedge valve_open_status[4]) begin
    if (reset_completed) begin
        valve4_seen_open = 1'b1;
        valve4_open_tick = u_controller.current_time;
    end
end

always @(negedge valve_open_status[4]) begin
    if (reset_completed && valve4_seen_open)
        valve4_close_tick = u_controller.current_time;
end

always @(negedge clk) begin
    if (reset_completed && (u_controller.fsm_state == 4'd1) &&
        ((valve_open_status & ~64'h10) != 0)) begin
        $display("RESET_OLD_SCHEDULE_REAPPEARED tick=%0d status=%016x",
                 u_controller.current_time, valve_open_status);
        failures = failures + 1;
        reset_completed = 1'b0;
    end
end

initial begin
    #25000000;
    $fatal(1, "SCHEDULER_RESET_TIMEOUT tick=%0d status=%016x count=%0d",
           u_controller.current_time, valve_open_status,
           u_controller.u_schedule_table.active_count);
end

initial begin
    repeat (5) @(negedge clk);
    rstn = 1'b1;

    // Mix active and future records from separate UDP packets.
    send_udp_open(0, 0, 2, 18);  // valve 0: [2,20) ms
    send_udp_open(1, 1, 10, 10); // valve 1: [10,20) ms, future at reset
    send_udp_open(2, 2, 3, 5);   // valve 2: [3,8) ms
    send_udp_open(3, 3, 0, 20);  // valve 3: [0,20) ms

    wait ((u_controller.current_time == 40) &&
          (u_controller.fsm_state == 4'd1));
    if (valve_open_status[3:0] !== 4'b1101) begin
        $display("RESET_PRECONDITION_FAIL status=%x expected=d",
                 valve_open_status[3:0]);
        failures = failures + 1;
    end

    // Bad application CRC must leave all four records untouched.
    send_udp_reset(1'b1);
    repeat (20) @(negedge clk);
    if ((reset_accept_count != 0) ||
        (u_controller.u_schedule_table.active_count != 4)) begin
        $display("RESET_BAD_CRC_EFFECT count=%0d active=%0d",
                 reset_accept_count,
                 u_controller.u_schedule_table.active_count);
        failures = failures + 1;
    end

    // Valid reset at 5 ms clears active and future records and closes valves.
    wait ((u_controller.current_time == 50) &&
          (u_controller.fsm_state == 4'd1));
    send_udp_reset(1'b0);
    wait (reset_accept_count == 1);
    wait (u_controller.fsm_state != 4'd1);
    wait ((u_controller.fsm_state == 4'd1) &&
          !u_controller.batch_active);

    if ((reset_accept_tick != 50) ||
        (u_controller.u_schedule_table.active_count != 0) ||
        (valve_open_status != 0) ||
        (u_controller.boost_time_setting != 16'd15) ||
        (u_controller.hold_duty_setting != 4'd5)) begin
        $display("RESET_EXECUTION_FAIL tick=%0d active=%0d status=%016x boost=%0d hold=%0d",
                 reset_accept_tick,
                 u_controller.u_schedule_table.active_count,
                 valve_open_status, u_controller.boost_time_setting,
                 u_controller.hold_duty_setting);
        failures = failures + 1;
    end
    reset_completed = 1'b1;

    // A new post-reset packet must work from a fresh schedule. Old valve 1
    // would otherwise open at 10 ms, making this a ghost-record check too.
    wait ((u_controller.current_time == 60) &&
          (u_controller.fsm_state == 4'd1));
    send_udp_open(4, 4, 2, 3); // absolute [8,11) ms

    wait ((u_controller.current_time >= 210) &&
          (u_controller.fsm_state == 4'd1));
    if ((valve4_open_tick != 80) || (valve4_close_tick != 110) ||
        (valve_open_status != 0) ||
        (u_controller.u_schedule_table.active_count != 0)) begin
        $display("RESET_POST_COMMAND_FAIL open=%0d close=%0d status=%016x active=%0d",
                 valve4_open_tick, valve4_close_tick, valve_open_status,
                 u_controller.u_schedule_table.active_count);
        failures = failures + 1;
    end

    if (failures == 0)
        $display("UDP_SCHEDULER_RESET_PASS reset_tick=5ms post_reset_valve4=8-11ms no_ghosts");
    else
        $fatal(1, "UDP_SCHEDULER_RESET_FAIL failures=%0d", failures);
    $finish;
end

endmodule
