`timescale 1ns/1ps

module tb_valve_command_batch;

logic        clk = 1'b0;
logic        rstn = 1'b0;
logic        command_valid = 1'b0;
logic [63:0] command_data = '0;
wire         command_ready;
wire         pwm_s1_wr_en;
wire [5:0]   pwm_s1_wr_addr;
wire [3:0]   pwm_s1_wr_duty;
wire         pwm_s2_wr_en;
wire [5:0]   pwm_s2_wr_addr;
wire [3:0]   pwm_s2_wr_duty;
wire [63:0]  valve_open_status;

integer failures = 0;
integer valve_ticks = 0;
logic   count_ticks = 1'b0;

always #5 clk = ~clk;

valve_controller #(
    // Five thousand clocks per 0.1 ms logical timer tick leaves time to scan
    // all schedule and valve RAM records while keeping this test short.
    .CLK_FREQ_HZ (500_000),
    .TIMER_HZ    (100),
    .VALVE_COUNT (64),
    .PWM_LEVELS  (10),
    .SCHEDULE_DEPTH (1024)
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

always @(posedge clk) begin
    if (count_ticks && dut.scan_advances_time &&
        (dut.fsm_state == 4'd5) && (dut.work_index == 6'd3))
        valve_ticks = valve_ticks + 1;
end

task automatic send_open(
    input [15:0] delay_ms,
    input [15:0] duration_ms,
    input logic final_in_packet
);
    begin
        wait (command_ready);
        @(negedge clk);
        command_data = {
            2'd1,
            6'd3,
            6'd3,
            delay_ms,
            duration_ms,
            final_in_packet,
            17'd0
        };
        command_valid = 1'b1;
        @(negedge clk);
        command_valid = 1'b0;
    end
endtask

initial begin
    #10000000;
    $fatal(1, "VALVE_BATCH_TIMEOUT ticks=%0d count=%0b state=%0d work=%0d batch=%0b pending=%0b open=%0b",
           valve_ticks, count_ticks, dut.fsm_state, dut.work_index,
           dut.batch_active, dut.timer_pending, valve_open_status[3]);
end

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (command_ready);

    // Exact protocol example. Inspect the three retained intervals directly
    // so this test does not need to simulate ten seconds of wall-clock time.
    send_open(16'd5000, 16'd5000, 1'b0);
    send_open(16'd3000, 16'd5000, 1'b0);
    send_open(16'd6000, 16'd2000, 1'b1);

    wait (!dut.batch_active && (dut.fsm_state == 4'd1));
    if (dut.u_schedule_table.active_count !== 3 ||
        (dut.u_schedule_table.schedule_memory[0][63:32] -
         dut.batch_epoch) !== 32'd50000 ||
        (dut.u_schedule_table.schedule_memory[0][31:0] -
         dut.batch_epoch) !== 32'd100000 ||
        (dut.u_schedule_table.schedule_memory[1][63:32] -
         dut.batch_epoch) !== 32'd30000 ||
        (dut.u_schedule_table.schedule_memory[1][31:0] -
         dut.batch_epoch) !== 32'd80000 ||
        (dut.u_schedule_table.schedule_memory[2][63:32] -
         dut.batch_epoch) !== 32'd60000 ||
        (dut.u_schedule_table.schedule_memory[2][31:0] -
         dut.batch_epoch) !== 32'd80000) begin
        $display("VALVE_BATCH_SECONDS_SCHEDULE_FAIL");
        failures = failures + 1;
    end

    // Reset, then run a millisecond-scaled version through the timer to verify
    // that the union of the three retained intervals has two transitions.
    @(negedge clk);
    rstn = 1'b0;
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (command_ready);

    // Scaled version of the specified example:
    // [5,10] ms, [3,8] ms and [6,8] ms => [3,10] ms.
    send_open(16'd5, 16'd5, 1'b0);
    send_open(16'd3, 16'd5, 1'b0);
    send_open(16'd6, 16'd2, 1'b1);

    wait (!dut.batch_active && (dut.fsm_state == 4'd1));
    if (dut.u_schedule_table.active_count !== 3) begin
        $display("VALVE_BATCH_SCHEDULE_FAIL count=%0d",
                 dut.u_schedule_table.active_count);
        failures = failures + 1;
    end

    valve_ticks = 0;
    count_ticks = 1'b1;
    wait (valve_open_status[3]);
    if (valve_ticks != 30) begin
        $display("VALVE_BATCH_DELAY_FAIL ticks=%0d expected=30", valve_ticks);
        failures = failures + 1;
    end

    wait (!valve_open_status[3]);
    if (valve_ticks != 100) begin
        $display("VALVE_BATCH_DURATION_FAIL ticks=%0d expected=100",
                 valve_ticks);
        failures = failures + 1;
    end

    if (failures == 0)
        $display("VALVE_COMMAND_BATCH_PASS delay_s=3 duration_s=7 transitions=pass");
    else
        $fatal(1, "VALVE_COMMAND_BATCH_FAIL failures=%0d", failures);
    $finish;
end

endmodule
