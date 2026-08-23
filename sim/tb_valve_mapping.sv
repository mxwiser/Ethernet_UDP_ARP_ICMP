`timescale 1ns/1ps

module tb_valve_mapping;

localparam integer VALVE_COUNT = 64;

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

integer valve_number;
integer write_number;
integer expected_group;
integer expected_address;
integer expected_q;
integer error_count = 0;

always #5 clk = ~clk;

valve_controller #(
    // Keep the periodic timer out of this short mapping test.
    .CLK_FREQ_HZ (1_000_000),
    .TIMER_HZ    (1),
    .VALVE_COUNT (VALVE_COUNT),
    .PWM_LEVELS  (10)
) dut (
    .clk            (clk),
    .rstn           (rstn),
    .command_valid  (command_valid),
    .command_data   (command_data),
    .command_ready  (command_ready),
    .pwm_s1_wr_en   (pwm_s1_wr_en),
    .pwm_s1_wr_addr (pwm_s1_wr_addr),
    .pwm_s1_wr_duty (pwm_s1_wr_duty),
    .pwm_s2_wr_en   (pwm_s2_wr_en),
    .pwm_s2_wr_addr (pwm_s2_wr_addr),
    .pwm_s2_wr_duty (pwm_s2_wr_duty)
);

function automatic integer expected_a_q(input integer c_index);
    begin
        case (c_index)
            0: expected_a_q = 0;
            1: expected_a_q = 4;
            2: expected_a_q = 3;
            default: expected_a_q = 7;
        endcase
    end
endfunction

function automatic integer expected_b_q(input integer c_index);
    begin
        case (c_index)
            0: expected_b_q = 1;
            1: expected_b_q = 2;
            2: expected_b_q = 5;
            default: expected_b_q = 6;
        endcase
    end
endfunction

task automatic check_write(
    input integer expected_serial,
    input integer expected_channel,
    input integer expected_duty,
    input integer logical_valve,
    input integer is_b
);
    begin
        @(negedge clk);

        if (expected_serial == 1) begin
            if (!pwm_s1_wr_en || pwm_s2_wr_en ||
                (pwm_s1_wr_addr !== expected_channel[5:0]) ||
                (pwm_s1_wr_duty !== expected_duty[3:0])) begin
                $error("valve %0d %s: expected S1 Q%0d duty %0d, got S1(en=%0b addr=%0d duty=%0d) S2(en=%0b addr=%0d duty=%0d)",
                       logical_valve, is_b ? "B" : "A",
                       expected_channel, expected_duty,
                       pwm_s1_wr_en, pwm_s1_wr_addr, pwm_s1_wr_duty,
                       pwm_s2_wr_en, pwm_s2_wr_addr, pwm_s2_wr_duty);
                error_count = error_count + 1;
            end
        end else begin
            if (pwm_s1_wr_en || !pwm_s2_wr_en ||
                (pwm_s2_wr_addr !== expected_channel[5:0]) ||
                (pwm_s2_wr_duty !== expected_duty[3:0])) begin
                $error("valve %0d %s: expected S2 Q%0d duty %0d, got S1(en=%0b addr=%0d duty=%0d) S2(en=%0b addr=%0d duty=%0d)",
                       logical_valve, is_b ? "B" : "A",
                       expected_channel, expected_duty,
                       pwm_s1_wr_en, pwm_s1_wr_addr, pwm_s1_wr_duty,
                       pwm_s2_wr_en, pwm_s2_wr_addr, pwm_s2_wr_duty);
                error_count = error_count + 1;
            end
        end
    end
endtask

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;

    // Initialization clears all 64 valve-state RAM entries.
    wait (command_ready);

    for (valve_number = 0;
         valve_number < VALVE_COUNT;
         valve_number = valve_number + 1) begin
        expected_group = (valve_number < 32) ? 1 : 2;

        // COMMAND_OPEN, start=end=valve_number, zero delay, 1.0 ms duration.
        @(negedge clk);
        command_data = {
            2'd1,
            valve_number[5:0],
            valve_number[5:0],
            16'd0,
            16'd1,
            18'd0
        };
        command_valid = 1'b1;
        @(negedge clk);
        command_valid = 1'b0;

        expected_q = expected_a_q(valve_number % 4);
        expected_address = ((valve_number % 32) / 4) * 8 + expected_q;
        check_write(expected_group, expected_address, 10,
                    valve_number, 0);

        expected_q = expected_b_q(valve_number % 4);
        expected_address = ((valve_number % 32) / 4) * 8 + expected_q;
        check_write(expected_group, expected_address, 10,
                    valve_number, 1);

        wait (command_ready);
    end

    if (error_count == 0)
        $display("PASS: all 64 user valves map to the expected S1/S2 A/B outputs");
    else
        $fatal(1, "FAIL: %0d mapping errors", error_count);

    $finish;
end

endmodule
