`timescale 1ns/1ps

module tb_udp_500_packet_full_pressure;

localparam integer PACKET_COUNT = 500;
localparam integer SCHEDULE_DEPTH = 1024;

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
integer packet_index;
integer byte_index;
integer command_index;
integer expected_count = 0;
integer expected_rejects = 0;
integer accepted_packets = 0;
integer accepted_records = 0;
integer reset_packets = 0;
integer max_retained = 0;
integer failures = 0;

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
    .CLK_FREQ_HZ(1_000_000), .TIMER_HZ(10),
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

function automatic integer packet_command_count(input integer index);
    integer epoch_index;
    begin
        epoch_index = index % 125;
        if (epoch_index < 10)
            packet_command_count = 100;
        else if (epoch_index == 10)
            packet_command_count = 24;
        else
            packet_command_count = ((index * 37 + 11) % 100) + 1;
    end
endfunction

task automatic append_crc(input integer data_length);
    reg [31:0] crc;
    begin
        crc = 32'hffff_ffff;
        for (byte_index = 0; byte_index < data_length;
             byte_index = byte_index + 1)
            crc = crc32_byte(crc, payload[byte_index]);
        crc = crc ^ 32'hffff_ffff;
        payload[data_length+0] = crc[31:24];
        payload[data_length+1] = crc[23:16];
        payload[data_length+2] = crc[15:8];
        payload[data_length+3] = crc[7:0];
        payload_length = data_length + 4;
    end
endtask

task automatic build_open_packet(
    input integer index,
    input integer command_count
);
    integer base;
    integer first_valve;
    integer last_valve;
    integer delay_ms;
    integer duration_ms;
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h01;
        payload[2] = command_count;

        for (command_index = 0; command_index < command_count;
             command_index = command_index + 1) begin
            base = 3 + command_index * 6;
            first_valve = (index * 17 + command_index * 13) % 64;
            last_valve = first_valve +
                         ((index * 5 + command_index * 3) % 4);
            if (last_valve > 63)
                last_valve = 63;
            delay_ms = 50_000 + ((index * 29 + command_index * 7) % 5_000);
            duration_ms = 5_000 + ((index * 19 + command_index * 11) % 5_000);

            payload[base+0] = first_valve;
            payload[base+1] = last_valve;
            payload[base+2] = delay_ms >> 8;
            payload[base+3] = delay_ms;
            payload[base+4] = duration_ms >> 8;
            payload[base+5] = duration_ms;
        end

        append_crc(3 + command_count * 6);
    end
endtask

task automatic build_reset_packet;
    begin
        payload[0] = 8'hff;
        payload[1] = 8'h03;
        append_crc(2);
    end
endtask

task automatic transmit_current_packet;
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

initial begin
    #100_000_000;
    $fatal(1, "UDP500_FULL_TIMEOUT packet=%0d count=%0d rejects=%0d",
           packet_index, u_controller.schedule_active_count,
           u_controller.rejected_packet_count);
end

initial begin
    integer command_count;
    integer epoch_index;

    repeat (4) @(negedge clk);
    rstn = 1'b1;
    wait (controller_ready);

    for (packet_index = 0; packet_index < PACKET_COUNT;
         packet_index = packet_index + 1) begin
        epoch_index = packet_index % 125;

        if (epoch_index == 124) begin
            build_reset_packet();
            transmit_current_packet();
            wait (scheduler_reset_pulse);
            wait_pipeline_idle();
            expected_count = 0;
            reset_packets = reset_packets + 1;
        end else begin
            command_count = packet_command_count(packet_index);
            build_open_packet(packet_index, command_count);
            transmit_current_packet();
            wait_pipeline_idle();

            if (expected_count + command_count <= SCHEDULE_DEPTH) begin
                expected_count = expected_count + command_count;
                accepted_packets = accepted_packets + 1;
                accepted_records = accepted_records + command_count;
            end else begin
                expected_rejects = expected_rejects + 1;
            end
        end

        if (u_controller.schedule_active_count != expected_count) begin
            $display("UDP500_FULL_COUNT_FAIL packet=%0d got=%0d expected=%0d",
                     packet_index, u_controller.schedule_active_count,
                     expected_count);
            failures = failures + 1;
        end
        if (u_controller.rejected_packet_count != expected_rejects) begin
            $display("UDP500_FULL_REJECT_FAIL packet=%0d got=%0d expected=%0d",
                     packet_index, u_controller.rejected_packet_count,
                     expected_rejects);
            failures = failures + 1;
        end
        if (u_controller.schedule_active_count > max_retained)
            max_retained = u_controller.schedule_active_count;
        if (valve_open_status !== 64'd0) begin
            $display("UDP500_FULL_EARLY_OPEN_FAIL packet=%0d status=%016x",
                     packet_index, valve_open_status);
            failures = failures + 1;
        end
    end

    if ((accepted_packets != 44) || (accepted_records != 4096) ||
        (expected_rejects != 452) || (reset_packets != 4) ||
        (max_retained != 1024) || (expected_count != 0)) begin
        $display("UDP500_FULL_TOTAL_FAIL accepted_packets=%0d records=%0d rejects=%0d resets=%0d max=%0d final=%0d",
                 accepted_packets, accepted_records, expected_rejects,
                 reset_packets, max_retained, expected_count);
        failures = failures + 1;
    end

    $display("UDP500_FULL_RESULT packets=500 accepted_packets=%0d accepted_records=%0d rejected_packets=%0d resets=%0d",
             accepted_packets, accepted_records, expected_rejects,
             reset_packets);
    $display("UDP500_FULL_RESULT max_retained=%0d final_count=%0d final_status=%016x",
             max_retained, u_controller.schedule_active_count,
             valve_open_status);

    if (failures == 0)
        $display("UDP_500_PACKET_FULL_PRESSURE_PASS");
    else
        $fatal(1, "UDP_500_PACKET_FULL_PRESSURE_FAIL failures=%0d", failures);
    $finish;
end

endmodule
