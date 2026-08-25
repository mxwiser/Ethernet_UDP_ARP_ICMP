`timescale 1ns/1ps
`include "hc595.svh"

// Reproduce the observable consequence of an electrical disturbance on the
// LED-board 74HC595 chain after all valves have stopped.  RTL simulation
// cannot calculate connector arcing, ground bounce, or a supply brownout, so
// the disturbance is represented by corruption of the external 595 storage
// register.  The test then checks that the 1 kHz refresh repairs it.
module tb_led_hotplug_fault;

localparam integer LED_COUNT = 128;

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

logic [LED_COUNT-1:0] shift_register = '0;
logic [LED_COUNT-1:0] output_register = '0;
integer latch_count = 0;
integer latch_count_before_idle;
integer error_count = 0;
time fault_time;
time recovery_time;

// 50 MHz, matching the production system clock.
always #10 clk = ~clk;

hc595 led_bus();

valve_controller #(
    // One timer tick per 1000 clocks keeps the 64-entry scan realistic while
    // keeping the test short. One command millisecond equals ten timer ticks.
    .CLK_FREQ_HZ (1_000),
    .TIMER_HZ    (1),
    .VALVE_COUNT (64),
    .PWM_LEVELS  (10),
    .SCHEDULE_DEPTH (256)
) u_valve_controller (
    .clk              (clk),
    .rstn             (rstn),
    .scheduler_reset  (1'b0),
    .command_valid    (command_valid),
    .command_data     (command_data),
    .command_ready    (command_ready),
    .pwm_s1_wr_en     (pwm_s1_wr_en),
    .pwm_s1_wr_addr   (pwm_s1_wr_addr),
    .pwm_s1_wr_duty   (pwm_s1_wr_duty),
    .pwm_s2_wr_en     (pwm_s2_wr_en),
    .pwm_s2_wr_addr   (pwm_s2_wr_addr),
    .pwm_s2_wr_duty   (pwm_s2_wr_duty),
    .valve_open_status(valve_open_status)
);

HC595LED #(
    .CHIP_NUMBERS  (16),
    .CLK_FREQ_HZ   (50_000_000),
    .SHIFT_CLK_HZ  (1_000_000),
    .REFRESH_HZ    (1_000),
    .LED_ACTIVE_LOW(1'b0)
) u_hc595_led (
    .clk              (clk),
    .rstn             (rstn),
    .valve_open_status(valve_open_status),
    .hc595_led        (led_bus)
);

// Behavioral model of the 16 cascaded LED-board 74HC595 devices.
always @(posedge led_bus.shcp)
    shift_register <= {shift_register[LED_COUNT-2:0], led_bus.ser};

always @(posedge led_bus.stcp) begin
    output_register <= shift_register;
    latch_count = latch_count + 1;
end

task automatic send_command(input logic [63:0] payload);
    begin
        wait (command_ready);
        @(negedge clk);
        command_data  = payload;
        command_valid = 1'b1;
        @(negedge clk);
        command_valid = 1'b0;
    end
endtask

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (command_ready);
    @(posedge led_bus.stcp);
    #1;

    // Exercise one real work/stop cycle before injecting the disturbance.
    send_command({
        2'd1, 6'd3, 6'd3, 16'd0, 16'd1, 1'b1, 17'd0
    });
    wait (valve_open_status[3]);
    wait (!valve_open_status[3]);
    wait ((u_hc595_led.state == 2'd3) &&
          (u_hc595_led.latched_pattern == '0) &&
          (led_bus.stcp == 1'b0));
    repeat (10) @(posedge clk);

    if (valve_open_status !== '0 || output_register !== '0) begin
        $error("pre-fault: stop did not leave valves and LEDs off");
        error_count = error_count + 1;
    end

    latch_count_before_idle = latch_count;

    // Electrical fault abstraction: a hot-plug transient corrupts only the
    // external LED-chain storage register. The FPGA's logical valve state is
    // still all zero, matching the report that no valve actually blew air.
    output_register = {LED_COUNT{1'b1}};
    fault_time = $time;
    $display("FAULT_INJECTED: external LED register changed to all ones");

    // No command is sent. The next 1 kHz periodic transfer must rewrite the
    // all-off logical pattern and repair the external storage register.
    wait (latch_count > latch_count_before_idle);
    recovery_time = $time;
    #1;

    if (valve_open_status !== '0) begin
        $error("post-fault: a logical valve unexpectedly opened");
        error_count = error_count + 1;
    end
    if (output_register !== '0) begin
        $error("post-fault: periodic refresh did not restore all-off LEDs");
        error_count = error_count + 1;
    end
    if ((recovery_time - fault_time) > 1_000_000) begin
        $error("post-fault: recovery exceeded one 1 kHz refresh period");
        error_count = error_count + 1;
    end

    $display("FAULT_RECOVERED: valves=off, LEDs restored in %0d ns",
             recovery_time - fault_time);

    // Consecutive idle latches must be exactly 1 ms apart at a 50 MHz clock.
    @(posedge led_bus.stcp);
    if (($time - recovery_time) != 1_000_000) begin
        $error("refresh cadence is %0d ns, expected 1 ms",
               $time - recovery_time);
        error_count = error_count + 1;
    end

    if (error_count == 0)
        $display("PASS: 1 kHz refresh repairs an all-on LED-register disturbance");
    else
        $fatal(1, "FAIL: %0d hot-plug fault-model errors", error_count);

    $finish;
end

endmodule
