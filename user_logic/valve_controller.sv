module valve_controller #(
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer TIMER_HZ    = 10_000,
    parameter integer VALVE_COUNT = 64,
    parameter integer PWM_LEVELS  = 10
)(
    input  wire         clk,
    input  wire         rstn,

    input  wire         command_valid,
    input  wire [63:0]  command_data,
    output wire         command_ready,

    output logic        pwm_s1_wr_en,
    output logic [5:0]  pwm_s1_wr_addr,
    output logic [3:0]  pwm_s1_wr_duty,
    output logic        pwm_s2_wr_en,
    output logic [5:0]  pwm_s2_wr_addr,
    output logic [3:0]  pwm_s2_wr_duty
);

localparam logic [1:0] COMMAND_OPEN = 2'd1;
localparam logic [1:0] COMMAND_SET  = 2'd2;
localparam integer TIMER_CYCLES = CLK_FREQ_HZ / TIMER_HZ;
localparam integer TIMER_COUNT_WIDTH =
    (TIMER_CYCLES <= 1) ? 1 : $clog2(TIMER_CYCLES);
localparam integer TIME_WIDTH = 20;
localparam integer STATE_WIDTH = 57;

localparam logic [3:0] FULL_DUTY = 4'd10;

localparam logic [3:0] STATE_INITIALIZE    = 4'd0;
localparam logic [3:0] STATE_IDLE          = 4'd1;
localparam logic [3:0] STATE_APPLY_WAIT    = 4'd2;
localparam logic [3:0] STATE_APPLY_PROCESS = 4'd3;
localparam logic [3:0] STATE_TIMER_WAIT    = 4'd4;
localparam logic [3:0] STATE_TIMER_PROCESS = 4'd5;
localparam logic [3:0] STATE_SET_WAIT      = 4'd6;
localparam logic [3:0] STATE_SET_PROCESS   = 4'd7;
localparam logic [3:0] STATE_PWM_SECOND    = 4'd8;

wire [1:0]  command_opcode  = command_data[63:62];
wire [5:0]  command_start   = command_data[61:56];
wire [5:0]  command_end     = command_data[55:50];
wire [15:0] command_param_0 = command_data[49:34];
wire [15:0] command_param_1 = command_data[33:18];

wire [TIME_WIDTH-1:0] command_param_0_extended =
    {{(TIME_WIDTH-16){1'b0}}, command_param_0};
wire [TIME_WIDTH-1:0] command_param_1_extended =
    {{(TIME_WIDTH-16){1'b0}}, command_param_1};
wire [TIME_WIDTH-1:0] command_delay_ticks =
    (command_param_0_extended << 3) + (command_param_0_extended << 1);
wire [TIME_WIDTH-1:0] command_duration_ticks =
    (command_param_1_extended << 3) + (command_param_1_extended << 1);

logic [3:0] fsm_state;
logic [TIMER_COUNT_WIDTH-1:0] timer_divider;
logic                         timer_pending;

logic [5:0]            work_index;
logic [5:0]            work_end;
logic [TIME_WIDTH-1:0] apply_delay_ticks;
logic [TIME_WIDTH-1:0] apply_duration_ticks;

logic [15:0] boost_time_setting;
logic [3:0]  hold_duty_setting;

// One packed state word per valve:
// [56] open, [55:36] delay remaining (0.1 ms),
// [35:16] open time remaining (0.1 ms), [15:0] boost elapsed (0.1 ms).
// A synchronous RAM avoids building a large variable-index multiplexer from
// thousands of individual timer registers.
(* ramstyle = "M9K" *) logic [STATE_WIDTH-1:0]
    valve_state_memory [0:VALVE_COUNT-1];
logic [5:0]                 memory_read_address;
logic [STATE_WIDTH-1:0]     memory_read_data;
logic                       memory_write_enable;
logic [5:0]                 memory_write_address;
logic [STATE_WIDTH-1:0]     memory_write_data;

wire                        state_open      = memory_read_data[56];
wire [TIME_WIDTH-1:0]       state_delay     = memory_read_data[55:36];
wire [TIME_WIDTH-1:0]       state_remaining = memory_read_data[35:16];
wire [15:0]                 state_boost     = memory_read_data[15:0];
wire [15:0]                 next_boost =
    (state_boost == 16'hFFFF) ? 16'hFFFF : state_boost + 1'b1;

wire apply_overwrites = apply_duration_ticks > state_remaining;
wire apply_starts_now = (fsm_state == STATE_APPLY_PROCESS) &&
                        apply_overwrites && !state_open &&
                        (apply_delay_ticks == 0);
wire timer_closes = (fsm_state == STATE_TIMER_PROCESS) &&
                    state_open && (state_remaining <= 1);
wire timer_opens = (fsm_state == STATE_TIMER_PROCESS) &&
                   !state_open && (state_delay == 1);
wire timer_leaves_boost = (fsm_state == STATE_TIMER_PROCESS) &&
                          state_open && (state_remaining > 1) &&
                          (state_boost < boost_time_setting) &&
                          (next_boost >= boost_time_setting);

logic       second_pwm_is_b;
logic [3:0] second_pwm_duty;
logic       second_return_to_apply;

logic       pwm_write_enable;
logic [5:0] pwm_write_valve;
logic       pwm_write_is_b;
logic [3:0] pwm_write_duty;

assign command_ready = (fsm_state == STATE_IDLE) && !timer_pending;
assign memory_read_address = work_index;

// --------------------------------------------------------------------------
// PCB mapping section
// --------------------------------------------------------------------------
// 用户层阀门编号按照 PCB 上从右向左的物理顺序排列：
//   0～31  -> 右侧 S1 的 8 片 74HC595
//   32～63 -> 左侧 S2 的 8 片 74HC595
// 每一串的芯片以及每片芯片上的 c0～c3 均从右向左排列。
// HC595PWM 通道 0 对应最右侧第一片芯片的 QA，通道 8 对应下一片的 QA。
//
// 每片 74HC595 的 PCB 接线：
//                  c0       c1       c2       c3
//   反向电动势 A： Q0       Q4       Q3       Q7
//   PWM 控制 B：   Q1       Q2       Q5       Q6
function automatic logic map_valve_to_group(input logic [5:0] valve_number);
    // 编号 0～31 选择 S1，编号 32～63 选择 S2。
    map_valve_to_group = valve_number[5];
endfunction

function automatic [5:0] map_valve_to_a_channel(
    input logic [5:0] valve_number
);
    logic [2:0] q_index;
    begin
        case (valve_number[1:0])
            2'd0: q_index = 3'd0; // c0 A -> Q0
            2'd1: q_index = 3'd4; // c1 A -> Q4
            2'd2: q_index = 3'd3; // c2 A -> Q3
            default: q_index = 3'd7; // c3 A -> Q7
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
            2'd0: q_index = 3'd1; // c0 B -> Q1
            2'd1: q_index = 3'd2; // c1 B -> Q2
            2'd2: q_index = 3'd5; // c2 B -> Q5
            default: q_index = 3'd6; // c3 B -> Q6
        endcase

        map_valve_to_b_channel = {valve_number[4:2], q_index};
    end
endfunction

// Infer one synchronous-read, synchronous-write memory. It is explicitly
// cleared by STATE_INITIALIZE after reset, because resetting every RAM bit
// asynchronously would force the timers back into logic cells.
always_ff @(posedge clk) begin
    memory_read_data <= valve_state_memory[memory_read_address];
    if (memory_write_enable)
        valve_state_memory[memory_write_address] <= memory_write_data;
end

always_comb begin
    memory_write_enable  = 1'b0;
    memory_write_address = work_index;
    memory_write_data    = memory_read_data;

    case (fsm_state)
        STATE_INITIALIZE: begin
            memory_write_enable = 1'b1;
            memory_write_data   = '0;
        end

        STATE_APPLY_PROCESS: begin
            if (apply_overwrites) begin
                memory_write_enable = 1'b1;
                if (state_open) begin
                    // An open valve ignores the new delay and keeps its
                    // present boost phase; only the final close is extended.
                    memory_write_data = {
                        1'b1,
                        {TIME_WIDTH{1'b0}},
                        apply_duration_ticks,
                        state_boost
                    };
                end else begin
                    memory_write_data = {
                        (apply_delay_ticks == 0),
                        apply_delay_ticks,
                        apply_duration_ticks,
                        16'd0
                    };
                end
            end
        end

        STATE_TIMER_PROCESS:begin 
            if (state_open) begin 
                memory_write_enable = 1'b1;
                if (state_remaining <= 1) begin
                    memory_write_data = '0;
                end else begin
                    memory_write_data = {
                        1'b1,
                        {TIME_WIDTH{1'b0}},
                        state_remaining - 1'b1,
                        next_boost
                    };
                end
            end else if (state_delay != 0) begin
                memory_write_enable = 1'b1;
                if (state_delay == 1) begin
                    memory_write_data = {
                        1'b1,
                        {TIME_WIDTH{1'b0}},
                        state_remaining,
                        16'd0
                    };
                end else begin
                    memory_write_data = {
                        1'b0,
                        state_delay - 1'b1,
                        state_remaining,
                        16'd0
                    };
                end
            end
        end

        default: begin
        end
    endcase
end

// 把一次“逻辑阀门写入”转换成对应595组的PWM写端口。
always_comb begin
    pwm_write_enable = 1'b0;
    pwm_write_valve  = work_index;
    pwm_write_is_b   = 1'b0;
    pwm_write_duty   = 4'd0;

    if (apply_starts_now) begin
        // Turn A on before writing B.
        pwm_write_enable = 1'b1;
        pwm_write_duty   = FULL_DUTY;
    end else if (timer_opens) begin
        // Turn A on before writing B.
        pwm_write_enable = 1'b1;
        pwm_write_duty   = FULL_DUTY;
    end else if (timer_closes) begin
        // Stop PWM before turning the flyback-control output A off.
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = 1'b1;
        pwm_write_duty   = 4'd0;
    end else if (timer_leaves_boost) begin
        pwm_write_enable = 1'b1;
        pwm_write_is_b   = 1'b1;
        pwm_write_duty   = hold_duty_setting;
    end else if ((fsm_state == STATE_SET_PROCESS) && state_open) begin
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
        // map_valve_to_group返回0时写hc595_s1，返回1时写hc595_s2。
        if (!map_valve_to_group(pwm_write_valve)) begin
            pwm_s1_wr_en = 1'b1;
            // pwm_write_is_b=1表示写B端，否则写A端。
            if (pwm_write_is_b)
                pwm_s1_wr_addr = map_valve_to_b_channel(pwm_write_valve);
            else
                pwm_s1_wr_addr = map_valve_to_a_channel(pwm_write_valve);
        end else begin
            pwm_s2_wr_en = 1'b1;
            // s2组内的通道编号同样从Q0开始计算。
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
        work_index             <= '0;
        work_end               <= '0;
        apply_delay_ticks      <= '0;
        apply_duration_ticks   <= '0;
        boost_time_setting     <= 16'd15;
        hold_duty_setting      <= 4'd5;
        second_pwm_is_b        <= 1'b0;
        second_pwm_duty        <= '0;
        second_return_to_apply <= 1'b0;
    end else begin
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
                if (timer_pending) begin
                    timer_pending <= 1'b0;
                    work_index    <= '0;
                    work_end      <= 6'd63;
                    fsm_state     <= STATE_TIMER_WAIT;
                end else if (command_valid) begin
                    if (command_opcode == COMMAND_SET) begin
                        boost_time_setting <= command_param_0;
                        hold_duty_setting  <= command_param_1[3:0];
                        work_index         <= '0;
                        work_end           <= 6'd63;
                        fsm_state          <= STATE_SET_WAIT;
                    end else if (command_opcode == COMMAND_OPEN) begin
                        work_index           <= command_start;
                        work_end             <= command_end;
                        apply_delay_ticks    <= command_delay_ticks;
                        apply_duration_ticks <= command_duration_ticks;
                        fsm_state            <= STATE_APPLY_WAIT;
                    end
                end
            end

            STATE_APPLY_WAIT:
                fsm_state <= STATE_APPLY_PROCESS;

            STATE_APPLY_PROCESS: begin
                if (apply_starts_now) begin
                    second_pwm_is_b        <= 1'b1;
                    second_pwm_duty        <=
                        (boost_time_setting == 0) ?
                            hold_duty_setting : FULL_DUTY;
                    second_return_to_apply <= 1'b1;
                    fsm_state              <= STATE_PWM_SECOND;
                end else if (work_index == work_end) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_APPLY_WAIT;
                end
            end

            STATE_TIMER_WAIT:
                fsm_state <= STATE_TIMER_PROCESS;

            STATE_TIMER_PROCESS: begin
                if (timer_opens) begin
                    second_pwm_is_b        <= 1'b1;
                    second_pwm_duty        <=
                        (boost_time_setting == 0) ?
                            hold_duty_setting : FULL_DUTY;
                    second_return_to_apply <= 1'b0;
                    fsm_state              <= STATE_PWM_SECOND;
                end else if (timer_closes) begin
                    second_pwm_is_b        <= 1'b0;
                    second_pwm_duty        <= 4'd0;
                    second_return_to_apply <= 1'b0;
                    fsm_state              <= STATE_PWM_SECOND;
                end else if (work_index == work_end) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_TIMER_WAIT;
                end
            end

            STATE_SET_WAIT:
                fsm_state <= STATE_SET_PROCESS;

            STATE_SET_PROCESS: begin
                if (work_index == work_end) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    fsm_state  <= STATE_SET_WAIT;
                end
            end

            STATE_PWM_SECOND: begin
                if (work_index == work_end) begin
                    fsm_state <= STATE_IDLE;
                end else begin
                    work_index <= work_index + 1'b1;
                    if (second_return_to_apply)
                        fsm_state <= STATE_APPLY_WAIT;
                    else
                        fsm_state <= STATE_TIMER_WAIT;
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
