`timescale 1ns/1ps

module tb_pll_areset_generator;

localparam integer INPUT_CLK_FREQ_HZ = 40_000;
localparam integer RESET_TIME_MS = 1;
localparam integer EXPECTED_RESET_CYCLES = 40;

logic clkin = 1'b0;
wire  areset;
integer rising_edge_count = 0;
integer error_count = 0;

// Scaled 40 kHz input clock: 25 us per cycle. The reset duration is still
// verified as exactly INPUT_CLK_FREQ_HZ / 1000 cycles per millisecond.
always #12_500 clkin = ~clkin;

pll_areset_generator #(
    .INPUT_CLK_FREQ_HZ(INPUT_CLK_FREQ_HZ),
    .RESET_TIME_MS    (RESET_TIME_MS)
) dut (
    .clkin (clkin),
    .areset(areset)
);

initial begin
    if (areset !== 1'b1) begin
        $error("power-up: PLL areset was not asserted");
        error_count = error_count + 1;
    end

    while (areset) begin
        @(posedge clkin);
        rising_edge_count = rising_edge_count + 1;
        #1;
    end

    if (rising_edge_count != EXPECTED_RESET_CYCLES) begin
        $error("release: got %0d reset cycles, expected %0d",
               rising_edge_count, EXPECTED_RESET_CYCLES);
        error_count = error_count + 1;
    end

    repeat (10) begin
        @(posedge clkin);
        #1;
        if (areset !== 1'b0) begin
            $error("steady-state: PLL areset reasserted");
            error_count = error_count + 1;
        end
    end

    if (error_count == 0)
        $display("PASS: PLL areset held for %0d clkin cycles then stayed low",
                 EXPECTED_RESET_CYCLES);
    else
        $fatal(1, "FAIL: %0d PLL-reset errors", error_count);

    $finish;
end

endmodule
