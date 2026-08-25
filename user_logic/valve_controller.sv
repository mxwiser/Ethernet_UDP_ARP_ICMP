module valve_controller #(
    parameter integer CLK_FREQ_HZ   = 50_000_000,
    parameter integer TIMER_HZ      = 10_000,
    parameter integer VALVE_COUNT   = 64,
    parameter integer PWM_LEVELS    = 10,
    parameter integer SCHEDULE_DEPTH = 1024
)(
    input  wire         clk,
    input  wire         rstn,
    input  wire         scheduler_reset,

    input  wire         command_valid,
    input  wire [63:0]  command_data,
    output wire         command_ready,

    output logic        pwm_s1_wr_en,
    output logic [5:0]  pwm_s1_wr_addr,
    output logic [3:0]  pwm_s1_wr_duty,
    output logic        pwm_s2_wr_en,
    output logic [5:0]  pwm_s2_wr_addr,
    output logic [3:0]  pwm_s2_wr_duty,

    // One bit per user valve. High throughout both boost and hold phases.
    output logic [VALVE_COUNT-1:0] valve_open_status
);

localparam logic [1:0] COMMAND_OPEN = 2'd1;
localparam logic [1:0] COMMAND_SET  = 2'd2;
localparam logic [1:0] COMMAND_RESET = 2'd3;
localparam integer TIMER_CYCLES = CLK_FREQ_HZ / TIMER_HZ;
localparam integer TIMER_COUNT_WIDTH =
    (TIMER_CYCLES <= 1) ? 1 : $clog2(TIMER_CYCLES);
localparam logic [3:0] FULL_DUTY = 4'd10;

localparam logic [3:0] STATE_INITIALIZE       = 4'd0;
localparam logic [3:0] STATE_IDLE             = 4'd1;
localparam logic [3:0] STATE_SCHEDULE_WAIT    = 4'd2;
localparam logic [3:0] STATE_SCHEDULE_PROCESS = 4'd3;
localparam logic [3:0] STATE_VALVE_WAIT       = 4'd4;
localparam logic [3:0] STATE_VALVE_PROCESS    = 4'd5;
localparam logic [3:0] STATE_SET_WAIT         = 4'd6;
localparam logic [3:0] STATE_SET_PROCESS      = 4'd7;
localparam logic [3:0] STATE_PWM_SECOND       = 4'd8;

wire [1:0]  command_opcode  = command_data[63:62];
wire [5:0]  command_start   = command_data[61:56];
wire [5:0]  command_end     = command_data[55:50];
wire [15:0] command_param_0 = command_data[49:34];
wire [15:0] command_param_1 = command_data[33:18];
wire        command_batch_last = command_data[17];
wire [7:0]  command_packet_count_field = command_data[16:9];
wire [31:0] command_packet_count =
    (command_packet_count_field == 0) ?
        32'd1 : {24'd0, command_packet_count_field};

wire [31:0] command_delay_ticks =
    ({16'd0, command_param_0} << 3) +
    ({16'd0, command_param_0} << 1);
wire [31:0] command_duration_ticks =
    ({16'd0, command_param_1} << 3) +
    ({16'd0, command_param_1} << 1);

logic [3:0] fsm_state;
logic [TIMER_COUNT_WIDTH-1:0] timer_divider;
logic                         timer_pending;
logic [31:0]                  current_time;
logic [31:0]                  scan_time;
logic                         scan_advances_time;

logic [15:0] boost_time_setting;
logic [3:0]  hold_duty_setting;

logic schedule_scan_start;
wire  schedule_scan_busy;
wire  schedule_scan_done;
wire  schedule_append_ready;
wire  schedule_full;
wire  [VALVE_COUNT-1:0] schedule_active_mask;
wire  [$clog2(SCHEDULE_DEPTH+1)-1:0] schedule_active_count;

logic        batch_active;
logic [31:0] batch_epoch;
logic        drop_batch;
logic [31:0] rejected_packet_count;
logic        schedule_refresh_pending;
wire [31:0] command_epoch = batch_active ? batch_epoch : current_time;
wire [31:0] command_start_time = command_epoch + command_delay_ticks;
wire [31:0] command_end_time =
    command_start_time + command_duration_ticks;
wire command_accepted =
    (fsm_state == STATE_IDLE) && command_valid && command_ready;
wire schedule_clear =
    scheduler_reset ||
    (command_accepted && (command_opcode == COMMAND_RESET));
wire [31:0] schedule_count_extended = schedule_active_count;
wire open_packet_fits =
    (schedule_count_extended + command_packet_count <= SCHEDULE_DEPTH);
wire open_command_will_append =
    (command_opcode == COMMAND_OPEN) && !drop_batch &&
    (batch_active || open_packet_fits);

logic [VALVE_COUNT-1:0] desired_open_status;

logic [5:0] work_index;

// Boost elapsed time remains one small RAM entry per valve. Scheduling is
// kept separately so one valve can own multiple disjoint future intervals.
(* ramstyle = "M9K" *) logic [15:0]
    valve_state_memory [0:VALVE_COUNT-1];
logic [15:0] valve_read_data;
logic        valve_write_enable;
logic [5:0]  valve_write_address;
logic [15:0] valve_write_data;

wire desired_open = desired_open_status[work_index];
wire valve_is_open = valve_open_status[work_index];
wire [15:0] state_boost = valve_read_data;
wire [15:0] next_boost =
    (state_boost == 16'hffff) ? 16'hffff : state_boost + 1'b1;

wire valve_opens = (fsm_state == STATE_VALVE_PROCESS) &&
                   desired_open && !valve_is_open;
wire valve_closes = (fsm_state == STATE_VALVE_PROCESS) &&
                    !desired_open && valve_is_open;
wire valve_leaves_boost = (fsm_state == STATE_VALVE_PROCESS) &&
                          desired_open && valve_is_open &&
                          (state_boost < boost_time_setting) &&
                          (next_boost >= boost_time_setting);

logic       second_pwm_is_b;
logic [3:0] second_pwm_duty;

logic       pwm_write_enable;
logic [5:0] pwm_write_valve;
logic       pwm_write_is_b;
logic [3:0] pwm_write_duty;

assign command_ready =
    (fsm_state == STATE_IDLE) &&
    (!timer_pending || batch_active || drop_batch) &&
    (!schedule_refresh_pending || batch_active || drop_batch) &&
    ((command_opcode == COMMAND_SET) ||
     (command_opcode == COMMAND_RESET) ||
     drop_batch ||
     ((command_opcode == COMMAND_OPEN) &&
      (!batch_active && !open_packet_fits)) ||
     schedule_append_ready);

valve_schedule_table #(
    .SCHEDULE_DEPTH (SCHEDULE_DEPTH),
    .VALVE_COUNT    (VALVE_COUNT)
) u_schedule_table (
    .clk                (clk),
    .rstn               (rstn),
    .clear              (schedule_clear),
    .append_valid       (command_accepted && open_command_will_append),
    .append_ready       (schedule_append_ready),
    .append_start_valve (command_start),
    .append_end_valve   (command_end),
    .append_start_time  (command_start_time),
    .append_end_time    (command_end_time),
    .scan_start         (schedule_scan_start),
    .scan_time          (scan_time),
    .scan_busy          (schedule_scan_busy),
    .scan_done          (schedule_scan_done),
    .active_valve_mask  (schedule_active_mask),
    .full               (schedule_full),
    .active_count       (schedule_active_count)
);

always_ff @(posedge clk) begin
    valve_read_data <= valve_state_memory[work_index];
    if (valve_write_enable)
        valve_state_memory[valve_write_address] <= valve_write_data;
end

always_comb begin
    valve_write_enable  = 1'b0;
    valve_write_address = work_index;
    valve_write_data    = valve_read_data;

    if (fsm_state == STATE_INITIALIZE) begin
        valve_write_enable = 1'b1;
        valve_write_data   = 16'd0;
    end else if (fsm_state == STATE_VALVE_PROCESS) begin
        if (valve_opens || valve_closes) begin
            valve_write_enable = 1'b1;
            valve_write_data   = 16'd0;
        end else if (desired_open && valve_is_open) begin
            valve_write_enable = 1'b1;
            valve_write_data   = next_boost;
        end
    end
end

// --------------------------------------------------------------------------
// PCB mapping section
// --------------------------------------------------------------------------
// User valves 0..31 use S1 and valves 32..63 use S2. Each group contains
// eight 74HC595 devices. The A/B channel wiring within each device is fixed
// by the PCB layout below.
function automatic logic map_valve_to_group(input logic [5:0] valve_number);
    map_valve_to_group = valve_number[5];
endfunction

function automatic [5:0] map_valve_to_a_channel(
    input logic [5:0] valve_number
);
    logic [2:0] q_index;
    begin
        case (valve_number[1:0])
            2'd0: q_index = 3'd0;
            2'd1: q_index = 3'd4;
            2'd2: q_index = 3'd3;
            default: q_index = 3'd7;
        endcase
        map_valve_to_a_channel = {valve_number[4:2], q_index};
    end
endfunction

function automatic [5:0] map_valve_to_b_channel(
    input logic [5:0] valve_number
);
    logic [2:0] q_index;
    begin
        case (valve_number[1:0])
            2'd0: q_index = 3'd1;
            2'd1: q_index = 3'd2;
            2'd2: q_index = 3'd5;
            default: q_index = 3'd6;
        endcase
        map_valve_to_b_channel = {valve_number[4:2], q_index};
    end
endfunction

always_comb begin
    pwm_write_enable = 1'b0;
    pwm_write_valve  = work_index;
    pwm_write_is_b   = 1'b0;
    pwm_write_duty   = 4'd0;

    if (valve_opens) begin
        // Turn A on before writing B.
        pwm_write_enable = 1'b1;
        pwm_write_duty   = FULL_DUTY;
    end else if (valve_closes) begin
        // Stop PWM before turning the flyback-control output A off.
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = 1'b1;
        pwm_write_duty   = 4'd0;
    end else if (valve_leaves_boost) begin
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = 1'b1;
        pwm_write_duty   = hold_duty_setting;
    end else if ((fsm_state == STATE_SET_PROCESS) && valve_is_open) begin
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = 1'b1;
        if (state_boost < boost_time_setting)
            pwm_write_duty = FULL_DUTY;
        else
            pwm_write_duty = hold_duty_setting;
    end else if (fsm_state == STATE_PWM_SECOND) begin
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = second_pwm_is_b;
        pwm_write_duty   = second_pwm_duty;
    end

    pwm_s1_wr_en   = 1'b0;
    pwm_s1_wr_addr = 6'd0;
    pwm_s1_wr_duty = pwm_write_duty;
    pwm_s2_wr_en   = 1'b0;
    pwm_s2_wr_addr = 6'd0;
    pwm_s2_wr_duty = pwm_write_duty;

    if (pwm_write_enable) begin
        if (!map_valve_to_group(pwm_write_valve)) begin
            pwm_s1_wr_en = 1'b1;
            if (pwm_write_is_b)
                pwm_s1_wr_addr = map_valve_to_b_channel(pwm_write_valve);
            else
                pwm_s1_wr_addr = map_valve_to_a_channel(pwm_write_valve);
        end else begin
            pwm_s2_wr_en = 1'b1;
            if (pwm_write_is_b)
                pwm_s2_wr_addr = map_valve_to_b_channel(pwm_write_valve);
            else
                pwm_s2_wr_addr = map_valve_to_a_channel(pwm_write_valve);
        end
    end
end

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        fsm_state              <= STATE_INITIALIZE;
        timer_divider          <= '0;
        timer_pending          <= 1'b0;
        current_time           <= 32'd0;
        scan_time              <= 32'd0;
        scan_advances_time     <= 1'b0;
        schedule_scan_start    <= 1'b0;
        boost_time_setting     <= 16'd15;
        hold_duty_setting      <= 4'd5;
        desired_open_status    <= '0;
        batch_active           <= 1'b0;
        batch_epoch            <= 32'd0;
        drop_batch             <= 1'b0;
        rejected_packet_count  <= 32'd0;
        schedule_refresh_pending <= 1'b0;
        work_index             <= '0;
        second_pwm_is_b        <= 1'b0;
        second_pwm_duty        <= '0;
        valve_open_status      <= '0;
    end else if (scheduler_reset) begin
        // This sideband bypasses the in-order command FIFO. It can therefore
        // reset a full scheduler even when an OPEN command is at the FIFO
        // head. PWM configuration and the monotonic time base are preserved.
        timer_divider            <= '0;
        timer_pending            <= 1'b0;
        scan_advances_time       <= 1'b0;
        schedule_scan_start      <= 1'b0;
        desired_open_status      <= '0;
        batch_active             <= 1'b0;
        drop_batch               <= 1'b0;
        schedule_refresh_pending <= 1'b0;
        work_index               <= '0;
        second_pwm_is_b          <= 1'b0;
        second_pwm_duty          <= '0;
        fsm_state                <= STATE_VALVE_WAIT;
    end else begin
        schedule_scan_start <= 1'b0;

        if (valve_opens)
            valve_open_status[work_index] <= 1'b1;
        else if (valve_closes)
            valve_open_status[work_index] <= 1'b0;

        if (timer_divider == TIMER_CYCLES - 1) begin
            timer_divider <= '0;
            timer_pending <= 1'b1;
        end else begin
            timer_divider <= timer_divider + 1'b1;
        end

        case (fsm_state)
            STATE_INITIALIZE: begin
                if (work_index == VALVE_COUNT - 1) begin
                    work_index <= '0;
                    fsm_state  <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                end
            end

            STATE_IDLE: begin
                if (schedule_refresh_pending && !batch_active) begin
                    schedule_refresh_pending <= 1'b0;
                    scan_advances_time  <= 1'b0;
                    scan_time           <= current_time;
                    schedule_scan_start <= 1'b1;
                    fsm_state           <= STATE_SCHEDULE_WAIT;
                end else if (timer_pending &&
                             !batch_active && !drop_batch) begin
                    timer_pending       <= 1'b0;
                    scan_advances_time  <= 1'b1;
                    current_time        <= current_time + 1'b1;
                    scan_time           <= current_time + 1'b1;
                    schedule_scan_start <= 1'b1;
                    fsm_state           <= STATE_SCHEDULE_WAIT;
                end else if (command_valid && command_ready) begin
                    if (command_opcode == COMMAND_SET) begin
                        batch_active       <= 1'b0;
                        drop_batch         <= 1'b0;
                        boost_time_setting <= command_param_0;
                        hold_duty_setting  <= command_param_1[3:0];
                        work_index         <= '0;
                        fsm_state          <= STATE_SET_WAIT;
                    end else if (command_opcode == COMMAND_RESET) begin
                        // Reset only the scheduler. PWM settings and the
                        // monotonic time base remain intact for later packets.
                        timer_divider            <= '0;
                        timer_pending            <= 1'b0;
                        scan_advances_time       <= 1'b0;
                        batch_active             <= 1'b0;
                        drop_batch               <= 1'b0;
                        schedule_refresh_pending <= 1'b0;
                        desired_open_status      <= '0;
                        work_index               <= '0;
                        fsm_state                <= STATE_VALVE_WAIT;
                    end else if (command_opcode == COMMAND_OPEN) begin
                        if (drop_batch) begin
                            if (command_batch_last)
                                drop_batch <= 1'b0;
                        end else if (!batch_active && !open_packet_fits) begin
                            // Capacity is reserved for the complete packet or
                            // none of it. Drain a rejected packet without ever
                            // appending a partial batch to the schedule table.
                            rejected_packet_count <=
                                rejected_packet_count + 1'b1;
                            drop_batch <= !command_batch_last;
                        end else begin
                            if (!batch_active)
                                batch_epoch <= current_time;
                            batch_active <= !command_batch_last;
                            if (command_batch_last)
                                schedule_refresh_pending <= 1'b1;
                        end
                    end
                end
            end

            STATE_SCHEDULE_WAIT: begin
                if (schedule_scan_done) begin
                    desired_open_status <= schedule_active_mask;
                    work_index <= '0;
                    fsm_state  <= STATE_VALVE_WAIT;
                end
            end

            STATE_VALVE_WAIT:
                fsm_state <= STATE_VALVE_PROCESS;

            STATE_VALVE_PROCESS: begin
                if (valve_opens) begin
                    second_pwm_is_b <= 1'b1;
                    second_pwm_duty <=
                        (boost_time_setting == 0) ?
                            hold_duty_setting : FULL_DUTY;
                    fsm_state <= STATE_PWM_SECOND;
                end else if (valve_closes) begin
                    second_pwm_is_b <= 1'b0;
                    second_pwm_duty <= 4'd0;
                    fsm_state <= STATE_PWM_SECOND;
                end else if (work_index == VALVE_COUNT - 1) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_VALVE_WAIT;
                end
            end

            STATE_SET_WAIT:
                fsm_state <= STATE_SET_PROCESS;

            STATE_SET_PROCESS: begin
                if (work_index == VALVE_COUNT - 1) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_SET_WAIT;
                end
            end

            STATE_PWM_SECOND: begin
                if (work_index == VALVE_COUNT - 1) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_VALVE_WAIT;
                end
            end

            default: begin
                work_index <= '0;
                fsm_state  <= STATE_INITIALIZE;
            end
        endcase
    end
end

endmodule
