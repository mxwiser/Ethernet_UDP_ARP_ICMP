`timescale 1ns/1ps

module tb_udp_500_packet_stress;

localparam integer PACKET_COUNT = 500;

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
logic generated_is_reset [0:PACKET_COUNT-1];
integer generated_start [0:PACKET_COUNT-1];
integer generated_end [0:PACKET_COUNT-1];
integer generated_delay_ms [0:PACKET_COUNT-1];
integer generated_duration_ms [0:PACKET_COUNT-1];
integer reference_start_tick [0:PACKET_COUNT-1];
integer reference_end_tick [0:PACKET_COUNT-1];

integer packet_index;
integer byte_index;
integer record_index;
integer valve_index;
integer accepted_count = 0;
integer failures = 0;
integer epoch_failures = 0;
integer mask_failures = 0;
integer scan_checks = 0;
integer max_end_tick = 0;
integer max_retained_records = 0;
integer max_active_records = 0;
integer active_records_at_scan;
integer generated_span;
integer reference_floor_index = 0;
integer reset_count = 0;
integer reset_clear_checks = 0;
logic reset_clear_pending = 1'b0;
logic [63:0] expected_mask_at_scan;
logic [63:0] valves_seen_open = 64'd0;
logic [3:0] previous_fsm_state = 4'd0;
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
    .command_ready    (!fifo_full),
    .command_valid    (parser_valid),
    .command_data     (parser_data)
);

command_fifo #(
    .DATA_WIDTH (64),
    .DEPTH      (128)
) u_fifo (
    .clk     (clk),
    .rstn    (rstn),
    .clear   (1'b0),
    .wr_en   (parser_valid),
    .wr_data (parser_data),
    .rd_en   (fifo_read),
    .rd_data (fifo_data),
    .empty   (fifo_empty),
    .full    (fifo_full)
);

assign fifo_read = !fifo_empty && controller_ready;

valve_controller #(
    // One scheduler tick is 0.1 ms. A packet is launched every tick, while
    // command delay and duration remain in the protocol's millisecond unit.
    .CLK_FREQ_HZ    (500_000),
    .TIMER_HZ       (100),
    .VALVE_COUNT    (64),
    .PWM_LEVELS     (10),
    .SCHEDULE_DEPTH (1024)
) u_controller (
    .clk               (clk),
    .rstn              (rstn),
    .scheduler_reset   (1'b0),
    .command_valid     (fifo_read),
    .command_data      (fifo_data),
    .command_ready     (controller_ready),
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
    integer bit_number;
    reg [31:0] crc;
    begin
        crc = crc_in ^ datum;
        for (bit_number = 0; bit_number < 8; bit_number = bit_number + 1)
            crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
        crc32_byte = crc;
    end
endfunction

function automatic [63:0] calculate_expected_mask(
    input [31:0] now_tick
);
    integer reference_index;
    integer reference_valve;
    begin
        calculate_expected_mask = 64'd0;
        for (reference_index = reference_floor_index;
             reference_index < accepted_count;
             reference_index = reference_index + 1) begin
            if (!generated_is_reset[reference_index] &&
                (now_tick >= reference_start_tick[reference_index]) &&
                (now_tick < reference_end_tick[reference_index])) begin
                for (reference_valve = generated_start[reference_index];
                     reference_valve <= generated_end[reference_index];
                     reference_valve = reference_valve + 1) begin
                    calculate_expected_mask[reference_valve] = 1'b1;
                end
            end
        end
    end
endfunction

task automatic send_udp_packet(input integer command_number);
    reg [31:0] crc;
    integer crc_data_length;
    begin
        payload[0] = 8'hff;
        if (generated_is_reset[command_number]) begin
            payload[1] = 8'h03;
            crc_data_length = 2;
        end else begin
            payload[1] = 8'h01;
            payload[2] = 8'd1;
            payload[3] = generated_start[command_number];
            payload[4] = generated_end[command_number];
            payload[5] = generated_delay_ms[command_number] >> 8;
            payload[6] = generated_delay_ms[command_number];
            payload[7] = generated_duration_ms[command_number] >> 8;
            payload[8] = generated_duration_ms[command_number];
            crc_data_length = 9;
        end

        crc = 32'hffff_ffff;
        for (byte_index = 0; byte_index < crc_data_length;
             byte_index = byte_index + 1)
            crc = crc32_byte(crc, payload[byte_index]);
        crc = crc ^ 32'hffff_ffff;
        payload[crc_data_length+0] = crc[31:24];
        payload[crc_data_length+1] = crc[23:16];
        payload[crc_data_length+2] = crc[15:8];
        payload[crc_data_length+3] = crc[7:0];

        wait (!u_parser.receiving && !u_parser.draining && !parser_valid);
        udp_rxamount = crc_data_length + 4;
        @(negedge clk);
        udp_rxstart = 1'b1;
        @(negedge clk);
        udp_rxstart = 1'b0;

        for (byte_index = 0; byte_index < crc_data_length + 4;
             byte_index = byte_index + 1) begin
            udp_rxdv   = 1'b1;
            udp_rxdata = payload[byte_index];
            udp_rxend  = (byte_index == crc_data_length + 3);
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

// Build the independent reference schedule from commands actually accepted
// by the controller. At the same time, verify parser/FIFO ordering and data.
always @(posedge clk) begin
    if (rstn && fifo_read) begin
        if (accepted_count >= PACKET_COUNT) begin
            $display("UDP500_EXTRA_COMMAND data=%016x", fifo_data);
            failures = failures + 1;
        end else begin
            if (generated_is_reset[accepted_count]) begin
                if (fifo_data !==
                    {2'd3, 6'd0, 6'd0, 16'd0, 16'd0,
                     1'b1, 8'd1, 9'd0}) begin
                    $display("UDP500_RESET_DECODE_FAIL packet=%0d got=%016x",
                             accepted_count, fifo_data);
                    failures = failures + 1;
                end
            end else begin
                if ((fifo_data[63:62] != 2'd1) ||
                    (fifo_data[61:56] != generated_start[accepted_count]) ||
                    (fifo_data[55:50] != generated_end[accepted_count]) ||
                    (fifo_data[49:34] !=
                        generated_delay_ms[accepted_count]) ||
                    (fifo_data[33:18] !=
                        generated_duration_ms[accepted_count]) ||
                    !fifo_data[17] || (fifo_data[16:9] != 8'd1)) begin
                    $display("UDP500_DECODE_FAIL packet=%0d got=%016x",
                             accepted_count, fifo_data);
                    failures = failures + 1;
                end
            end

            if (u_controller.current_time != accepted_count) begin
                epoch_failures = epoch_failures + 1;
                if (epoch_failures <= 5)
                    $display("UDP500_EPOCH_FAIL packet=%0d got=%0d expected=%0d",
                             accepted_count, u_controller.current_time,
                             accepted_count);
            end

            if (generated_is_reset[accepted_count]) begin
                reference_floor_index = accepted_count + 1;
                reset_count = reset_count + 1;
                reset_clear_pending = 1'b1;
            end else begin
                reference_start_tick[accepted_count] =
                    u_controller.current_time +
                    generated_delay_ms[accepted_count] * 10;
                reference_end_tick[accepted_count] =
                    reference_start_tick[accepted_count] +
                    generated_duration_ms[accepted_count] * 10;
                if (reference_end_tick[accepted_count] > max_end_tick)
                    max_end_tick = reference_end_tick[accepted_count];
            end
            accepted_count = accepted_count + 1;
        end
    end
end

// valve_open_status is stable whenever the controller has returned to IDLE.
// Compare all 64 valves after every refresh scan and every timer scan.
always @(negedge clk) begin
    if (!rstn) begin
        previous_fsm_state = 4'd0;
    end else begin
        if (reset_clear_pending) begin
            reset_clear_checks = reset_clear_checks + 1;
            if (u_controller.u_schedule_table.active_count != 0) begin
                $display("UDP500_RESET_CLEAR_FAIL packet=%0d active=%0d",
                         accepted_count - 1,
                         u_controller.u_schedule_table.active_count);
                failures = failures + 1;
            end
            reset_clear_pending = 1'b0;
        end

        if (monitor_enabled && (u_controller.fsm_state == 4'd1) &&
            (previous_fsm_state != 4'd1)) begin
            expected_mask_at_scan =
                calculate_expected_mask(u_controller.current_time);
            scan_checks = scan_checks + 1;

            active_records_at_scan = 0;
            for (record_index = reference_floor_index;
                 record_index < accepted_count;
                 record_index = record_index + 1) begin
                if (!generated_is_reset[record_index] &&
                    (u_controller.current_time >=
                     reference_start_tick[record_index]) &&
                    (u_controller.current_time <
                     reference_end_tick[record_index])) begin
                    active_records_at_scan = active_records_at_scan + 1;
                end
            end
            if (active_records_at_scan > max_active_records)
                max_active_records = active_records_at_scan;
            if (u_controller.u_schedule_table.active_count >
                max_retained_records)
                max_retained_records =
                    u_controller.u_schedule_table.active_count;

            valves_seen_open = valves_seen_open | valve_open_status;
            if (valve_open_status !== expected_mask_at_scan) begin
                mask_failures = mask_failures + 1;
                if (mask_failures <= 10)
                    $display("UDP500_MASK_FAIL check=%0d tick=%0d got=%016x expected=%016x",
                             scan_checks, u_controller.current_time,
                             valve_open_status, expected_mask_at_scan);
            end
        end
        previous_fsm_state = u_controller.fsm_state;
    end
end

initial begin
    #120000000;
    $fatal(1,
        "UDP500_TIMEOUT accepted=%0d tick=%0d max_end=%0d fifo_empty=%0b",
        accepted_count, u_controller.current_time, max_end_tick, fifo_empty);
end

initial begin
    repeat (5) @(negedge clk);
    rstn = 1'b1;
    monitor_enabled = 1'b1;

    // Deterministic, reproducible interleaving across all 64 valves.
    // Packets 199, 349, 350 and 474 are scheduler resets; 349/350 verifies
    // that consecutive reset commands are idempotent.
    for (packet_index = 0; packet_index < PACKET_COUNT;
         packet_index = packet_index + 1) begin
        generated_is_reset[packet_index] =
            (packet_index == 199) || (packet_index == 349) ||
            (packet_index == 350) || (packet_index == 474);
        generated_start[packet_index] =
            (packet_index * 17 + packet_index / 7) % 64;
        generated_span = (packet_index * 5 + 3) % 8;
        generated_end[packet_index] =
            generated_start[packet_index] + generated_span;
        if (generated_end[packet_index] > 63)
            generated_end[packet_index] = 63;
        generated_delay_ms[packet_index] =
            1 + ((packet_index * 37) % 60);
        generated_duration_ms[packet_index] =
            1 + ((packet_index * 23) % 25);

        // Launch one packet every 0.1 ms scheduler tick. Waiting for its
        // command to be accepted also proves that no parser/FIFO record is
        // silently lost under continuous multi-packet traffic.
        wait (u_controller.current_time >= packet_index);
        send_udp_packet(packet_index);
        wait (accepted_count == packet_index + 1);
    end

    wait (!u_parser.receiving && !u_parser.draining && !parser_valid &&
          fifo_empty && !u_controller.batch_active);
    wait ((u_controller.current_time > max_end_tick + 1) &&
          (u_controller.fsm_state == 4'd1) &&
          (u_controller.u_schedule_table.active_count == 0));

    if (accepted_count != PACKET_COUNT) begin
        $display("UDP500_COUNT_FAIL got=%0d expected=%0d",
                 accepted_count, PACKET_COUNT);
        failures = failures + 1;
    end
    if (epoch_failures != 0) begin
        $display("UDP500_EPOCH_FAILURES count=%0d", epoch_failures);
        failures = failures + 1;
    end
    if (mask_failures != 0) begin
        $display("UDP500_MASK_FAILURES count=%0d", mask_failures);
        failures = failures + 1;
    end
    if ((reset_count != 4) || (reset_clear_checks != 4)) begin
        $display("UDP500_RESET_COUNT_FAIL accepted=%0d clear_checks=%0d expected=4",
                 reset_count, reset_clear_checks);
        failures = failures + 1;
    end
    if (valves_seen_open !== 64'hffff_ffff_ffff_ffff) begin
        $display("UDP500_COVERAGE_FAIL valves_seen=%016x",
                 valves_seen_open);
        failures = failures + 1;
    end
    if (valve_open_status != 0) begin
        $display("UDP500_FINAL_STATUS_FAIL status=%016x",
                 valve_open_status);
        failures = failures + 1;
    end

    $display("UDP500_RESULT packets=%0d accepted=%0d resets=%0d scan_checks=%0d",
             PACKET_COUNT, accepted_count, reset_count, scan_checks);
    $display("UDP500_RESULT max_retained=%0d max_simultaneously_active=%0d valves_seen=%016x",
             max_retained_records, max_active_records, valves_seen_open);
    $display("UDP500_RESULT final_tick=%0d final_status=%016x",
             u_controller.current_time, valve_open_status);

    if (failures == 0)
        $display("UDP_500_PACKET_STRESS_PASS");
    else
        $fatal(1, "UDP_500_PACKET_STRESS_FAIL failures=%0d", failures);
    $finish;
end

endmodule
