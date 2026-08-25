`timescale 1ns/1ps

module tb_udp_multi_packet_interleaved;

logic clk = 1'b0;
logic rstn = 1'b0;
logic udp_rxstart = 1'b0;
logic udp_rxend = 1'b0;
logic udp_rxframe_done = 1'b0;
logic udp_rxdv = 1'b0;
logic [7:0] udp_rxdata = 8'd0;
logic [15:0] udp_rxamount = 16'd0;

wire parser_command_valid;
wire [63:0] parser_command_data;
wire command_fifo_empty;
wire command_fifo_full;
wire [63:0] command_fifo_data;
wire command_fifo_read;
wire controller_command_ready;
wire [63:0] valve_open_status;

wire pwm_s1_wr_en;
wire [5:0] pwm_s1_wr_addr;
wire [3:0] pwm_s1_wr_duty;
wire pwm_s2_wr_en;
wire [5:0] pwm_s2_wr_addr;
wire [3:0] pwm_s2_wr_duty;

logic [7:0] payload [0:1023];
integer payload_length;
integer failures = 0;
integer command_count;
integer valve_number;
integer transition_count [0:5];
integer transition_time [0:5][0:5];
logic   transition_state [0:5][0:5];
logic [5:0] previous_status = 6'd0;
logic monitor_enabled = 1'b0;

always #5 clk = ~clk;

udp_command_parser #(
    .MAX_COMMANDS (100)
) u_parser (
    .clk              (clk),
    .rstn             (rstn),
    .clear            (1'b0),
    .udp_rxstart      (udp_rxstart),
    .udp_rxend        (udp_rxend),
    .udp_rxframe_done (udp_rxframe_done),
    .udp_rxdv         (udp_rxdv),
    .udp_rxdata       (udp_rxdata),
    .udp_rxamount     (udp_rxamount),
    .command_ready    (!command_fifo_full),
    .command_valid    (parser_command_valid),
    .command_data     (parser_command_data)
);

command_fifo #(
    .DATA_WIDTH (64),
    .DEPTH      (128)
) u_command_fifo (
    .clk     (clk),
    .rstn    (rstn),
    .clear   (1'b0),
    .wr_en   (parser_command_valid),
    .wr_data (parser_command_data),
    .rd_en   (command_fifo_read),
    .rd_data (command_fifo_data),
    .empty   (command_fifo_empty),
    .full    (command_fifo_full)
);

assign command_fifo_read =
    !command_fifo_empty && controller_command_ready;

valve_controller #(
    // One logical timer tick is 0.1 ms. The 5000-cycle divider leaves ample
    // room for a worst-case 1024-record M9K scan between ticks.
    .CLK_FREQ_HZ    (500_000),
    .TIMER_HZ       (100),
    .VALVE_COUNT    (64),
    .PWM_LEVELS     (10),
    .SCHEDULE_DEPTH (1024)
) u_controller (
    .clk               (clk),
    .rstn              (rstn),
    .scheduler_reset   (1'b0),
    .command_valid     (command_fifo_read),
    .command_data      (command_fifo_data),
    .command_ready     (controller_command_ready),
    .pwm_s1_wr_en      (pwm_s1_wr_en),
    .pwm_s1_wr_addr    (pwm_s1_wr_addr),
    .pwm_s1_wr_duty    (pwm_s1_wr_duty),
    .pwm_s2_wr_en      (pwm_s2_wr_en),
    .pwm_s2_wr_addr    (pwm_s2_wr_addr),
    .pwm_s2_wr_duty    (pwm_s2_wr_duty),
    .valve_open_status (valve_open_status)
);

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

task automatic begin_open_packet(input [7:0] count);
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h01;
        payload[2] = count;
        command_count = count;
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

task automatic finish_packet;
    integer index;
    integer data_length;
    reg [31:0] crc;
    begin
        data_length = 3 + command_count * 6;
        crc = 32'hffff_ffff;
        for (index = 0; index < data_length; index = index + 1)
            crc = crc32_byte(crc, payload[index]);
        crc = crc ^ 32'hffff_ffff;
        payload[data_length+0] = crc[31:24];
        payload[data_length+1] = crc[23:16];
        payload[data_length+2] = crc[15:8];
        payload[data_length+3] = crc[7:0];
        payload_length = data_length + 4;
    end
endtask

task automatic transmit_packet;
    integer index;
    begin
        wait (!u_parser.receiving && !u_parser.draining &&
              !parser_command_valid);
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
        udp_rxframe_done = 1'b1;
        @(negedge clk);
        udp_rxframe_done = 1'b0;
    end
endtask

task automatic check_transition(
    input integer valve,
    input integer transition,
    input integer expected_time,
    input logic expected_state
);
    begin
        if ((transition_time[valve][transition] != expected_time) ||
            (transition_state[valve][transition] !== expected_state)) begin
            $display("INTERLEAVED_TRANSITION_FAIL valve=%0d n=%0d got=%0d/%0b expected=%0d/%0b",
                     valve, transition,
                     transition_time[valve][transition],
                     transition_state[valve][transition],
                     expected_time, expected_state);
            failures = failures + 1;
        end
    end
endtask

always @(valve_open_status[5:0]) begin
    if (monitor_enabled) begin
        for (valve_number = 0; valve_number < 6;
             valve_number = valve_number + 1) begin
            if (valve_open_status[valve_number] !=
                previous_status[valve_number]) begin
                if (transition_count[valve_number] < 6) begin
                    transition_time[valve_number]
                                   [transition_count[valve_number]] =
                        u_controller.current_time;
                    transition_state[valve_number]
                                    [transition_count[valve_number]] =
                        valve_open_status[valve_number];
                end
                transition_count[valve_number] =
                    transition_count[valve_number] + 1;
            end
        end
    end
    previous_status = valve_open_status[5:0];
end

initial begin
    #20000000;
    $fatal(1, "UDP_INTERLEAVED_TIMEOUT time=%0d status=%02x",
           u_controller.current_time, valve_open_status[5:0]);
end

initial begin
    for (valve_number = 0; valve_number < 6;
         valve_number = valve_number + 1)
        transition_count[valve_number] = 0;

    repeat (5) @(negedge clk);
    rstn = 1'b1;
    monitor_enabled = 1'b1;

    // UDP 1: valves 0..2 [2,6) ms; valve 4 [8,11) ms.
    begin_open_packet(2);
    put_open_command(0, 0, 2, 2, 4);
    put_open_command(1, 4, 4, 8, 3);
    finish_packet();
    transmit_packet();

    // UDP 2: valves 1..3 [4,9) ms; valve 5 [1,3) ms.
    begin_open_packet(2);
    put_open_command(0, 1, 3, 4, 5);
    put_open_command(1, 5, 5, 1, 2);
    finish_packet();
    transmit_packet();

    // UDP 3: valves 0..1 [7,9) ms; valves 3..5 [3,7) ms.
    begin_open_packet(2);
    put_open_command(0, 0, 1, 7, 2);
    put_open_command(1, 3, 5, 3, 4);
    finish_packet();
    transmit_packet();

    wait (!u_parser.draining && !parser_command_valid &&
          command_fifo_empty && !u_controller.batch_active &&
          (u_controller.fsm_state == 4'd1));

    // Send UDP 4 exactly 5 ms later. Its delays are relative to this packet:
    // valves 2..4 [7,10) ms absolute; valve 0 [10,12) ms absolute.
    wait ((u_controller.current_time == 50) &&
          (u_controller.fsm_state == 4'd1));
    begin_open_packet(2);
    put_open_command(0, 2, 4, 2, 3);
    put_open_command(1, 0, 0, 5, 2);
    finish_packet();
    transmit_packet();

    wait (!u_parser.draining && !parser_command_valid &&
          command_fifo_empty && !u_controller.batch_active);
    if (u_controller.batch_epoch != 50) begin
        $display("INTERLEAVED_PACKET_EPOCH_FAIL got=%0d expected=50",
                 u_controller.batch_epoch);
        failures = failures + 1;
    end

    wait ((u_controller.current_time >= 125) &&
          (u_controller.fsm_state == 4'd1));

    if (transition_count[0] != 6 || transition_count[1] != 2 ||
        transition_count[2] != 2 || transition_count[3] != 2 ||
        transition_count[4] != 2 || transition_count[5] != 2) begin
        $display("INTERLEAVED_COUNT_FAIL v0=%0d v1=%0d v2=%0d v3=%0d v4=%0d v5=%0d",
                 transition_count[0], transition_count[1],
                 transition_count[2], transition_count[3],
                 transition_count[4], transition_count[5]);
        failures = failures + 1;
    end

    check_transition(0, 0,  20, 1'b1);
    check_transition(0, 1,  60, 1'b0);
    check_transition(0, 2,  70, 1'b1);
    check_transition(0, 3,  90, 1'b0);
    check_transition(0, 4, 100, 1'b1);
    check_transition(0, 5, 120, 1'b0);
    check_transition(1, 0,  20, 1'b1);
    check_transition(1, 1,  90, 1'b0);
    check_transition(2, 0,  20, 1'b1);
    check_transition(2, 1, 100, 1'b0);
    check_transition(3, 0,  30, 1'b1);
    check_transition(3, 1, 100, 1'b0);
    check_transition(4, 0,  30, 1'b1);
    check_transition(4, 1, 110, 1'b0);
    check_transition(5, 0,  10, 1'b1);
    check_transition(5, 1,  70, 1'b0);

    if (valve_open_status[5:0] != 0) begin
        $display("INTERLEAVED_FINAL_STATUS_FAIL status=%02x",
                 valve_open_status[5:0]);
        failures = failures + 1;
    end

    $display("INTERLEAVED_RESULT v0=2-6,7-9,10-12ms v1=2-9ms v2=2-10ms");
    $display("INTERLEAVED_RESULT v3=3-10ms v4=3-11ms v5=1-7ms");
    if (failures == 0)
        $display("UDP_MULTI_PACKET_INTERLEAVED_PASS packets=4 commands=8");
    else
        $fatal(1, "UDP_MULTI_PACKET_INTERLEAVED_FAIL failures=%0d",
               failures);
    $finish;
end

endmodule
