`timescale 1ns/1ps

module tb_valve_schedule_table_capacity;

logic clk = 1'b0;
logic rstn = 1'b0;
logic clear = 1'b0;
logic append_valid = 1'b0;
wire  append_ready;
logic [5:0] append_start_valve = 6'd0;
logic [5:0] append_end_valve = 6'd0;
logic [31:0] append_start_time = 32'd0;
logic [31:0] append_end_time = 32'd0;
logic scan_start = 1'b0;
logic [31:0] scan_time = 32'd0;
wire scan_busy;
wire scan_done;
wire [63:0] active_valve_mask;
wire full;
wire [10:0] active_count;

integer index;
integer failures = 0;

always #5 clk = ~clk;

valve_schedule_table #(
    .SCHEDULE_DEPTH (1024),
    .VALVE_COUNT    (64)
) dut (
    .clk                (clk),
    .rstn               (rstn),
    .clear              (clear),
    .append_valid       (append_valid),
    .append_ready       (append_ready),
    .append_start_valve (append_start_valve),
    .append_end_valve   (append_end_valve),
    .append_start_time  (append_start_time),
    .append_end_time    (append_end_time),
    .scan_start         (scan_start),
    .scan_time          (scan_time),
    .scan_busy          (scan_busy),
    .scan_done          (scan_done),
    .active_valve_mask  (active_valve_mask),
    .full               (full),
    .active_count       (active_count)
);

task automatic append_record(
    input [5:0] valve,
    input [31:0] start_tick,
    input [31:0] end_tick
);
    begin
        wait (append_ready);
        @(negedge clk);
        append_start_valve = valve;
        append_end_valve   = valve;
        append_start_time  = start_tick;
        append_end_time    = end_tick;
        append_valid       = 1'b1;
        @(negedge clk);
        append_valid       = 1'b0;
    end
endtask

task automatic run_scan(input [31:0] now_tick);
    begin
        wait (!scan_busy);
        @(negedge clk);
        scan_time  = now_tick;
        scan_start = 1'b1;
        @(negedge clk);
        scan_start = 1'b0;
        wait (scan_done);
        @(negedge clk);
    end
endtask

initial begin
    #2000000;
    $fatal(1, "SCHEDULE_CAPACITY_TIMEOUT count=%0d busy=%0b",
           active_count, scan_busy);
end

initial begin
    repeat (4) @(negedge clk);
    rstn = 1'b1;

    // Fill all 1024 M9K records. Half expire at tick 50; half at tick 200.
    for (index = 0; index < 1024; index = index + 1) begin
        if (index < 512)
            append_record(6'd0, 32'd0, 32'd50);
        else
            append_record(6'd0, 32'd0, 32'd200);
    end

    if (!full || (active_count != 1024) || append_ready) begin
        $display("SCHEDULE_FILL_FAIL full=%0b count=%0d ready=%0b",
                 full, active_count, append_ready);
        failures = failures + 1;
    end

    // Remove the first 512 records and compact the retained half to 0..511.
    run_scan(32'd100);
    if (full || (active_count != 512) ||
        (active_valve_mask !== 64'h1) ||
        (dut.schedule_memory[0][75:70] != 6'd0) ||
        (dut.schedule_memory[511][31:0] != 32'd200)) begin
        $display("SCHEDULE_COMPACT_FAIL full=%0b count=%0d mask=%016x",
                 full, active_count, active_valve_mask);
        failures = failures + 1;
    end

    // Reuse all 512 reclaimed addresses and reach the 1024-record limit
    // again. These new entries belong to valve 1 and expire at tick 300.
    for (index = 0; index < 512; index = index + 1)
        append_record(6'd1, 32'd100, 32'd300);

    if (!full || (active_count != 1024)) begin
        $display("SCHEDULE_REFILL_FAIL full=%0b count=%0d",
                 full, active_count);
        failures = failures + 1;
    end

    run_scan(32'd250);
    if (full || (active_count != 512) ||
        (active_valve_mask !== 64'h2)) begin
        $display("SCHEDULE_REUSE_FAIL full=%0b count=%0d mask=%016x",
                 full, active_count, active_valve_mask);
        failures = failures + 1;
    end

    run_scan(32'd300);
    if (full || (active_count != 0) ||
        (active_valve_mask !== 64'h0)) begin
        $display("SCHEDULE_EMPTY_FAIL full=%0b count=%0d mask=%016x",
                 full, active_count, active_valve_mask);
        failures = failures + 1;
    end

    // A reset must remain effective at the hard 1024-record limit. The RAM
    // contents do not need a 1024-cycle erase because active_count makes the
    // old addresses unreachable immediately.
    for (index = 0; index < 1024; index = index + 1)
        append_record(index[5:0], 32'd400, 32'd1000);

    if (!full || (active_count != 1024)) begin
        $display("SCHEDULE_RESET_PREFILL_FAIL full=%0b count=%0d",
                 full, active_count);
        failures = failures + 1;
    end

    @(negedge clk);
    clear = 1'b1;
    @(negedge clk);
    clear = 1'b0;

    if (full || !append_ready || (active_count != 0) ||
        (active_valve_mask !== 64'h0) || scan_busy) begin
        $display("SCHEDULE_FULL_RESET_FAIL full=%0b ready=%0b count=%0d mask=%016x busy=%0b",
                 full, append_ready, active_count, active_valve_mask, scan_busy);
        failures = failures + 1;
    end

    if (failures == 0)
        $display("VALVE_SCHEDULE_CAPACITY_PASS fill=1024 compact=512 refill=1024 empty=0 full_reset=0");
    else
        $fatal(1, "VALVE_SCHEDULE_CAPACITY_FAIL failures=%0d", failures);
    $finish;
end

endmodule
