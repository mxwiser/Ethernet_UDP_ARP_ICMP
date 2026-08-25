`timescale 1ns/1ps

module tb_valve_led_status;

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

integer error_count = 0;

always #5 clk = ~clk;

valve_controller #(
    // A timer tick occurs every 1000 clocks, leaving enough time to scan all
    // 64 valve RAM entries between ticks while keeping simulation short.
    .CLK_FREQ_HZ (1_000),
    .TIMER_HZ    (1),
    .VALVE_COUNT (64),
    .PWM_LEVELS  (10),
    .SCHEDULE_DEPTH (256)
) dut (
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

task automatic set_pwm_parameters;
    begin
        // COMMAND_SET: boost for 2 timer ticks, then hold at duty 5/10.
        send_command({
            2'd2, 6'd0, 6'd0, 16'd2, 16'd5, 1'b1, 17'd0
        });
        wait (command_ready);
    end
endtask

task automatic open_valve(
    input logic [5:0] valve_number,
    input logic [15:0] delay_ms,
    input logic [15:0] duration_ms
);
    begin
        send_command({2'd1, valve_number, valve_number,
                      delay_ms, duration_ms, 1'b1, 17'd0});
    end
endtask

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (command_ready);

    if (valve_open_status !== '0) begin
        $error("reset: valve status is not all zero: %016h",
               valve_open_status);
        error_count = error_count + 1;
    end

    set_pwm_parameters();

    // Valve 3 starts immediately. Its LED status must cover both the initial
    // full-duty boost and the later hold-PWM phase.
    open_valve(6'd3, 16'd0, 16'd1);
    wait (valve_open_status[3]);

    wait (pwm_s1_wr_en && (pwm_s1_wr_addr == 6'd6) &&
          (pwm_s1_wr_duty == 4'd5));
    if (!valve_open_status[3]) begin
        $error("valve 3 LED status went low during hold PWM");
        error_count = error_count + 1;
    end

    wait (!valve_open_status[3]);

    // Valve 33 is delayed. Its indicator must remain off during the delay,
    // then stay on through boost/hold and turn off on automatic close.
    open_valve(6'd33, 16'd1, 16'd1);
    wait (command_ready);
    if (valve_open_status[33]) begin
        $error("valve 33 LED status turned on before its delay expired");
        error_count = error_count + 1;
    end

    wait (valve_open_status[33]);
    wait (pwm_s2_wr_en && (pwm_s2_wr_addr == 6'd2) &&
          (pwm_s2_wr_duty == 4'd5));
    if (!valve_open_status[33]) begin
        $error("valve 33 LED status went low during hold PWM");
        error_count = error_count + 1;
    end

    wait (!valve_open_status[33]);

    if (valve_open_status !== '0) begin
        $error("finish: unexpected open valve status %016h",
               valve_open_status);
        error_count = error_count + 1;
    end

    if (error_count == 0)
        $display("PASS: valve LED status covers boost/hold and clears on close");
    else
        $fatal(1, "FAIL: %0d valve-status errors", error_count);

    $finish;
end

endmodule
