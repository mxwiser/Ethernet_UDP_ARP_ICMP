`timescale 1ns/1ps
`include "hc595.svh"

module tb_hc595pwm_serial;

localparam integer CHANNEL_COUNT = 64;

logic       clk = 1'b0;
logic       rstn = 1'b0;
logic       pwm_wr_en = 1'b0;
logic [5:0] pwm_wr_addr = '0;
logic       pwm_wr_duty = 1'b0;
wire        init_done;

logic [CHANNEL_COUNT-1:0] physical_shift_register = '0;
logic [CHANNEL_COUNT-1:0] physical_output_register = '0;
logic [CHANNEL_COUNT-1:0] expected_outputs;

integer channel_number;
integer previous_channel;
integer error_count = 0;

always #5 clk = ~clk;

hc595 serial_bus();

HC595PWM #(
    .CHIP_NUMBERS (8),
    // These reduced simulation parameters preserve the complete 64-bit
    // transfer while shortening each PWM frame.
    .CLK_FREQ_HZ  (20_000),
    .PWM_FREQ_HZ  (100),
    .PWM_LEVELS   (1)
) dut (
    .rstn         (rstn),
    .clk          (clk),
    .pwm_wr_en    (pwm_wr_en),
    .pwm_wr_addr  (pwm_wr_addr),
    .pwm_wr_duty  (pwm_wr_duty),
    .init_done    (init_done),
    .hc595_serial (serial_bus)
);

// Behavioral model of eight cascaded 74HC595s.  Bit 0 is QA of the first
// chip connected to SER; bit 63 is QH of the eighth chip.
always @(posedge serial_bus.shcp)
    physical_shift_register <= {
        physical_shift_register[CHANNEL_COUNT-2:0],
        serial_bus.ser
    };

always @(posedge serial_bus.stcp)
    physical_output_register <= physical_shift_register;

task automatic write_duty(
    input integer channel,
    input integer duty
);
    begin
        @(negedge clk);
        pwm_wr_en   = 1'b1;
        pwm_wr_addr = channel[5:0];
        pwm_wr_duty = duty[0:0];
        @(negedge clk);
        pwm_wr_en   = 1'b0;
    end
endtask

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;

    // Check every HC595PWM address against the outputs of the serial-chain
    // model.  Only one physical Q output is active for each iteration.
    previous_channel = -1;
    for (channel_number = 0;
         channel_number < CHANNEL_COUNT;
         channel_number = channel_number + 1) begin
        if (previous_channel >= 0)
            write_duty(previous_channel, 0);
        write_duty(channel_number, 1);

        // A table write just before the PWM-frame boundary is intentionally
        // allowed for; three latch events make the check phase-independent.
        repeat (3) @(posedge serial_bus.stcp);
        #1;

        expected_outputs = 64'b1 << channel_number;
        if (!init_done || serial_bus.oen ||
            (physical_output_register !== expected_outputs)) begin
            $error("channel %0d: expected physical Q bitmap %016h, got %016h (init=%0b oen=%0b)",
                   channel_number, expected_outputs,
                   physical_output_register, init_done, serial_bus.oen);
            error_count = error_count + 1;
        end

        previous_channel = channel_number;
    end

    if (error_count == 0)
        $display("PASS: HC595PWM channels 0..63 reach first-chip QA through eighth-chip QH in order");
    else
        $fatal(1, "FAIL: %0d serial-chain mapping errors", error_count);

    $finish;
end

endmodule
