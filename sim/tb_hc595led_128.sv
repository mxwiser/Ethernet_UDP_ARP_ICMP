`timescale 1ns/1ps
`include "hc595.svh"

module tb_hc595led_128;

localparam integer LED_COUNT = 128;
localparam integer USER_LED_COUNT = 64;

logic clk = 1'b0;
logic rstn = 1'b0;
logic [USER_LED_COUNT-1:0] valve_open_status = '0;
logic [LED_COUNT-1:0] shift_register = '0;
logic [LED_COUNT-1:0] output_register = '0;
logic [LED_COUNT-1:0] expected_pattern;
integer led_number;
integer error_count = 0;

always #5 clk = ~clk;

hc595 led_bus();

HC595LED #(
    .CHIP_NUMBERS  (16),
    .CLK_FREQ_HZ   (2_000),
    .SHIFT_CLK_HZ  (1_000),
    // Keep periodic refresh away from the immediate-update mapping checks.
    .REFRESH_HZ    (1),
    .LED_ACTIVE_LOW(1'b0)
) dut (
    .clk       (clk),
    .rstn      (rstn),
    .valve_open_status (valve_open_status),
    .hc595_led (led_bus)
);

// Model 16 cascaded 74HC595s. Bit 0 is Q0 of the first chip at SER.
always @(posedge led_bus.shcp)
    shift_register <= {shift_register[LED_COUNT-2:0], led_bus.ser};

always @(posedge led_bus.stcp)
    output_register <= shift_register;

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;

    // The first transfer after reset must safely blank all LEDs.
    @(posedge led_bus.stcp);
    #1;
    if (led_bus.oen || (output_register !== '0)) begin
        $error("reset: expected all LEDs off, got %032h (oen=%0b)",
               output_register, led_bus.oen);
        error_count = error_count + 1;
    end

    for (led_number = 0;
         led_number < USER_LED_COUNT;
         led_number = led_number + 1) begin
        @(negedge clk);
        valve_open_status = 64'b1 << led_number;
        @(posedge led_bus.stcp);
        #1;

        expected_pattern =
            (128'b1 << led_number) |
            (128'b1 << (LED_COUNT - 1 - led_number));
        if (led_bus.oen || (output_register !== expected_pattern)) begin
            $error("step %0d: expected LED bitmap %032h, got %032h (oen=%0b)",
                   led_number, expected_pattern, output_register, led_bus.oen);
            error_count = error_count + 1;
        end
    end

    // Closing the final valve must clear both of its indicator LEDs.
    @(negedge clk);
    valve_open_status = '0;
    @(posedge led_bus.stcp);
    #1;
    if (output_register !== '0) begin
        $error("close: expected all LEDs off, got %032h", output_register);
        error_count = error_count + 1;
    end

    if (error_count == 0)
        $display("PASS: each of 64 valve states controls both matching PCB-row LEDs");
    else
        $fatal(1, "FAIL: %0d running-light errors", error_count);

    $finish;
end

endmodule
