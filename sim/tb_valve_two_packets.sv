`timescale 1ns/1ps

module tb_valve_two_packets;

logic clk = 1'b0;
logic rstn = 1'b0;
logic command_valid = 1'b0;
logic [63:0] command_data = '0;
wire command_ready;
wire pwm_s1_wr_en;
wire [5:0] pwm_s1_wr_addr;
wire [3:0] pwm_s1_wr_duty;
wire pwm_s2_wr_en;
wire [5:0] pwm_s2_wr_addr;
wire [3:0] pwm_s2_wr_duty;
wire [63:0] valve_open_status;

integer failures = 0;
integer timer_pass = 0;
integer valve0_open_pass = -1;
integer valve0_close_pass = -1;
integer valve1_open_pass [0:1];
integer valve1_close_pass [0:1];
integer valve1_open_count = 0;
integer valve1_close_count = 0;
logic count_passes = 1'b0;
logic valve0_seen_open = 1'b0;
logic [31:0] first_packet_epoch;
logic [31:0] second_packet_epoch;

always #5 clk = ~clk;

valve_controller #(
    // The test counts logical 0.1 ms timer passes. A deliberately short
    // divider keeps the two 25 ms scaled timelines fast in simulation.
    .CLK_FREQ_HZ   (500_000),
    .TIMER_HZ      (100),
    .VALVE_COUNT   (64),
    .PWM_LEVELS    (10),
    .SCHEDULE_DEPTH(1024)
) dut (
    .clk               (clk),
    .rstn              (rstn),
    .scheduler_reset   (1'b0),
    .command_valid     (command_valid),
    .command_data      (command_data),
    .command_ready     (command_ready),
    .pwm_s1_wr_en      (pwm_s1_wr_en),
    .pwm_s1_wr_addr    (pwm_s1_wr_addr),
    .pwm_s1_wr_duty    (pwm_s1_wr_duty),
    .pwm_s2_wr_en      (pwm_s2_wr_en),
    .pwm_s2_wr_addr    (pwm_s2_wr_addr),
    .pwm_s2_wr_duty    (pwm_s2_wr_duty),
    .valve_open_status (valve_open_status)
);

task automatic send_open(
    input [5:0] start_valve,
    input [5:0] end_valve,
    input [15:0] delay_ms,
    input [15:0] duration_ms,
    input logic final_in_packet
);
    begin
        wait (command_ready);
        @(negedge clk);
        command_data = {
            2'd1, start_valve, end_valve,
            delay_ms, duration_ms, final_in_packet, 17'd0
        };
        command_valid = 1'b1;
        @(negedge clk);
        command_valid = 1'b0;
    end
endtask

task automatic reset_controller;
    begin
        @(negedge clk);
        rstn = 1'b0;
        repeat (4) @(negedge clk);
        rstn = 1'b1;
        wait (command_ready);
    end
endtask

always @(posedge clk) begin
    if (count_passes && dut.scan_advances_time &&
        (dut.fsm_state == 4'd5) &&
        (dut.work_index == 6'd0)) begin
        timer_pass = timer_pass + 1;
    end
end

always @(posedge valve_open_status[0]) begin
    if (count_passes) begin
        valve0_seen_open = 1'b1;
        valve0_open_pass = timer_pass;
    end
end

always @(negedge valve_open_status[0]) begin
    if (count_passes && valve0_seen_open)
        valve0_close_pass = timer_pass;
end

always @(posedge valve_open_status[1]) begin
    if (count_passes && (valve1_open_count < 2)) begin
        valve1_open_pass[valve1_open_count] = timer_pass;
        valve1_open_count = valve1_open_count + 1;
    end
end

always @(negedge valve_open_status[1]) begin
    if (count_passes && (valve1_close_count < 2)) begin
        valve1_close_pass[valve1_close_count] = timer_pass;
        valve1_close_count = valve1_close_count + 1;
    end
end

initial begin
    #15000000;
    $fatal(1,
        "TWO_PACKET_TIMEOUT pass=%0d status=%02x open_count=%0d close_count=%0d",
        timer_pass, valve_open_status[1:0],
        valve1_open_count, valve1_close_count);
end

initial begin
    valve1_open_pass[0] = -1;
    valve1_open_pass[1] = -1;
    valve1_close_pass[0] = -1;
    valve1_close_pass[1] = -1;

    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (command_ready);

    // Exact protocol values in milliseconds.
    // UDP packet 1:
    send_open(6'd0, 6'd1, 16'd5000, 16'd5000, 1'b0);
    send_open(6'd0, 6'd1, 16'd2000, 16'd3000, 1'b1);
    first_packet_epoch = dut.batch_epoch;

    // UDP packet 2 follows immediately and has a separate epoch.
    send_open(6'd1, 6'd1, 16'd20000, 16'd5000, 1'b1);
    second_packet_epoch = dut.batch_epoch;
    wait (!dut.batch_active && (dut.fsm_state == 4'd1));

    if (dut.u_schedule_table.active_count !== 3 ||
        dut.u_schedule_table.schedule_memory[0][75:64] !==
            {6'd0, 6'd1} ||
        (dut.u_schedule_table.schedule_memory[0][63:32] -
         first_packet_epoch) !==
            32'd50000 ||
        (dut.u_schedule_table.schedule_memory[0][31:0] -
         first_packet_epoch) !==
            32'd100000 ||
        dut.u_schedule_table.schedule_memory[1][75:64] !==
            {6'd0, 6'd1} ||
        (dut.u_schedule_table.schedule_memory[1][63:32] -
         first_packet_epoch) !==
            32'd20000 ||
        (dut.u_schedule_table.schedule_memory[1][31:0] -
         first_packet_epoch) !==
            32'd50000 ||
        dut.u_schedule_table.schedule_memory[2][75:64] !==
            {6'd1, 6'd1} ||
        (dut.u_schedule_table.schedule_memory[2][63:32] -
         second_packet_epoch) !==
            32'd200000 ||
        (dut.u_schedule_table.schedule_memory[2][31:0] -
         second_packet_epoch) !==
            32'd250000) begin
        $display("TWO_PACKET_EXACT_SCHEDULE_FAIL");
        failures = failures + 1;
    end

    // Repeat at 1/1000 scale and observe every physical transition.
    reset_controller();
    send_open(6'd0, 6'd1, 16'd5, 16'd5, 1'b0);
    send_open(6'd0, 6'd1, 16'd2, 16'd3, 1'b1);
    send_open(6'd1, 6'd1, 16'd20, 16'd5, 1'b1);
    wait (!dut.batch_active && (dut.fsm_state == 4'd1));

    timer_pass = 0;
    valve0_seen_open = 1'b0;
    valve1_open_count = 0;
    valve1_close_count = 0;
    count_passes = 1'b1;
    wait ((valve0_close_pass >= 0) && (valve1_close_count == 2));

    $display("TRANSITIONS valve0 open=%0d close=%0d",
             valve0_open_pass, valve0_close_pass);
    $display("TRANSITIONS valve1 open=%0d close=%0d open=%0d close=%0d",
             valve1_open_pass[0], valve1_close_pass[0],
             valve1_open_pass[1], valve1_close_pass[1]);

    if (valve0_open_pass != 20 || valve0_close_pass != 100 ||
        valve1_open_pass[0] != 20 || valve1_close_pass[0] != 100 ||
        valve1_open_pass[1] != 200 || valve1_close_pass[1] != 250) begin
        failures = failures + 1;
    end

    if (failures == 0)
        $display("TWO_PACKET_PASS valve0=2s..10s valve1=2s..10s_and_20s..25s");
    else
        $fatal(1, "TWO_PACKET_FAIL failures=%0d", failures);
    $finish;
end

endmodule
