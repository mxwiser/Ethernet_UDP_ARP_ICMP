module valve_schedule_table #(
    parameter integer SCHEDULE_DEPTH = 1024,
    parameter integer VALVE_COUNT    = 64
)(
    input  wire         clk,
    input  wire         rstn,
    input  wire         clear,

    input  wire         append_valid,
    output wire         append_ready,
    input  wire [5:0]   append_start_valve,
    input  wire [5:0]   append_end_valve,
    input  wire [31:0]  append_start_time,
    input  wire [31:0]  append_end_time,

    input  wire         scan_start,
    input  wire [31:0]  scan_time,
    output logic        scan_busy,
    output logic        scan_done,
    output logic [VALVE_COUNT-1:0] active_valve_mask,
    output wire         full,
    output logic [$clog2(SCHEDULE_DEPTH+1)-1:0] active_count
);

localparam integer SCHEDULE_WIDTH = 76;
localparam integer POINTER_WIDTH =
    (SCHEDULE_DEPTH <= 2) ? 1 : $clog2(SCHEDULE_DEPTH);
localparam integer COUNT_WIDTH = $clog2(SCHEDULE_DEPTH + 1);

localparam logic [1:0] STATE_IDLE    = 2'd0;
localparam logic [1:0] STATE_WAIT    = 2'd1;
localparam logic [1:0] STATE_PROCESS = 2'd2;

logic [1:0] state;
logic [31:0] scan_time_latched;
logic [POINTER_WIDTH-1:0] read_index;
logic [COUNT_WIDTH-1:0] entries_remaining;
logic [COUNT_WIDTH-1:0] kept_count;

// Records are kept densely in addresses 0..active_count-1. Expired records
// are removed by compacting the table in place during a scan. Therefore the
// table needs no 1024-bit validity vector or resettable per-entry metadata.
// Record layout:
// [75:70] first valve, [69:64] last valve,
// [63:32] inclusive start tick, [31:0] exclusive end tick.
(* ramstyle = "M9K, no_rw_check" *) logic [SCHEDULE_WIDTH-1:0]
    schedule_memory [0:SCHEDULE_DEPTH-1];
logic [SCHEDULE_WIDTH-1:0] schedule_read_data;
logic schedule_write_enable;
logic [POINTER_WIDTH-1:0] schedule_write_address;
logic [SCHEDULE_WIDTH-1:0] schedule_write_data;

wire [5:0] record_start_valve = schedule_read_data[75:70];
wire [5:0] record_end_valve   = schedule_read_data[69:64];
wire [31:0] record_start_time = schedule_read_data[63:32];
wire [31:0] record_end_time   = schedule_read_data[31:0];

wire record_expired = time_reached(scan_time_latched, record_end_time);
wire record_active = time_reached(scan_time_latched, record_start_time) &&
                     !record_expired;
wire [VALVE_COUNT-1:0] record_valve_mask =
    make_valve_mask(record_start_valve, record_end_valve);

wire [COUNT_WIDTH-1:0] kept_count_after_record =
    kept_count + {{(COUNT_WIDTH-1){1'b0}}, !record_expired};

assign full = (active_count == SCHEDULE_DEPTH);
assign append_ready = (state == STATE_IDLE) && !scan_start && !full;

function automatic logic time_reached(
    input logic [31:0] now_tick,
    input logic [31:0] target_tick
);
    logic signed [31:0] difference;
    begin
        difference = now_tick - target_tick;
        time_reached = (difference >= 0);
    end
endfunction

function automatic [VALVE_COUNT-1:0] make_valve_mask(
    input logic [5:0] first_valve,
    input logic [5:0] last_valve
);
    integer valve_number;
    begin
        make_valve_mask = '0;
        for (valve_number = 0;
             valve_number < VALVE_COUNT;
             valve_number = valve_number + 1) begin
            if ((valve_number >= first_valve) &&
                (valve_number <= last_valve)) begin
                make_valve_mask[valve_number] = 1'b1;
            end
        end
    end
endfunction

always_comb begin
    schedule_write_enable  = 1'b0;
    schedule_write_address = '0;
    schedule_write_data    = '0;

    if ((state == STATE_IDLE) && append_valid && append_ready) begin
        schedule_write_enable  = 1'b1;
        schedule_write_address = active_count[POINTER_WIDTH-1:0];
        schedule_write_data    = {
            append_start_valve,
            append_end_valve,
            append_start_time,
            append_end_time
        };
    end else if ((state == STATE_PROCESS) && !record_expired &&
                 (kept_count[POINTER_WIDTH-1:0] != read_index)) begin
        // A previous expired record left a hole. Copy this retained record
        // toward address zero while the next source entry is read.
        schedule_write_enable  = 1'b1;
        schedule_write_address = kept_count[POINTER_WIDTH-1:0];
        schedule_write_data    = schedule_read_data;
    end
end

// This is the standard single-clock simple-dual-port inference template.
// Quartus maps a 1024 x 76 table to nine Cyclone IV M9K blocks.
always_ff @(posedge clk) begin
    schedule_read_data <= schedule_memory[read_index];
    if (schedule_write_enable)
        schedule_memory[schedule_write_address] <= schedule_write_data;
end

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state             <= STATE_IDLE;
        scan_time_latched <= 32'd0;
        read_index        <= '0;
        entries_remaining <= '0;
        kept_count        <= '0;
        scan_busy         <= 1'b0;
        scan_done         <= 1'b0;
        active_valve_mask <= '0;
        active_count      <= '0;
    end else if (clear) begin
        // Memory contents do not need clearing. With active_count at zero,
        // every old entry is unreachable and the next append overwrites it.
        state             <= STATE_IDLE;
        scan_time_latched <= 32'd0;
        read_index        <= '0;
        entries_remaining <= '0;
        kept_count        <= '0;
        scan_busy         <= 1'b0;
        scan_done         <= 1'b0;
        active_valve_mask <= '0;
        active_count      <= '0;
    end else begin
        scan_done <= 1'b0;

        case (state)
            STATE_IDLE: begin
                if (scan_start) begin
                    scan_time_latched <= scan_time;
                    read_index        <= '0;
                    entries_remaining <= active_count;
                    kept_count        <= '0;
                    active_valve_mask <= '0;

                    if (active_count == 0) begin
                        scan_busy <= 1'b0;
                        scan_done <= 1'b1;
                    end else begin
                        scan_busy <= 1'b1;
                        state     <= STATE_WAIT;
                    end
                end else if (append_valid && append_ready) begin
                    active_count <= active_count + 1'b1;
                end
            end

            STATE_WAIT: begin
                state <= STATE_PROCESS;
            end

            STATE_PROCESS: begin
                if (record_active)
                    active_valve_mask <=
                        active_valve_mask | record_valve_mask;

                kept_count <= kept_count_after_record;

                if (entries_remaining == 1) begin
                    active_count <= kept_count_after_record;
                    scan_busy    <= 1'b0;
                    scan_done    <= 1'b1;
                    state        <= STATE_IDLE;
                end else begin
                    entries_remaining <= entries_remaining - 1'b1;
                    read_index        <= read_index + 1'b1;
                    state             <= STATE_WAIT;
                end
            end

            default: begin
                state     <= STATE_IDLE;
                scan_busy <= 1'b0;
            end
        endcase
    end
end

endmodule
