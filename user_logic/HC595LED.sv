`include "hc595.svh"

module HC595LED #(
    parameter integer CHIP_NUMBERS = 16,
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer LED_STEP_MS = 100,
    parameter integer SHIFT_CLK_HZ = 1_000_000,
    // 0: a '1' turns an LED on; 1: a '0' turns an LED on.
    parameter bit LED_ACTIVE_LOW = 1'b0
)(
    input  logic clk,
    input  logic rstn,
    hc595.master hc595_led
);

localparam integer LED_COUNT = CHIP_NUMBERS * 8;
localparam integer ROW_LED_COUNT = LED_COUNT / 2;
localparam integer STEP_CYCLES_CALC =
    (CLK_FREQ_HZ / 1_000) * LED_STEP_MS;
localparam integer STEP_CYCLES =
    (STEP_CYCLES_CALC < 1) ? 1 : STEP_CYCLES_CALC;
localparam integer SHIFT_HALF_CYCLES_CALC =
    CLK_FREQ_HZ / (2 * SHIFT_CLK_HZ);
localparam integer SHIFT_HALF_CYCLES =
    (SHIFT_HALF_CYCLES_CALC < 1) ? 1 : SHIFT_HALF_CYCLES_CALC;

localparam integer STEP_COUNT_WIDTH =
    (STEP_CYCLES <= 1) ? 1 : $clog2(STEP_CYCLES);
localparam integer SHIFT_COUNT_WIDTH =
    (SHIFT_HALF_CYCLES <= 1) ? 1 : $clog2(SHIFT_HALF_CYCLES);
localparam integer LED_INDEX_WIDTH =
    (LED_COUNT <= 1) ? 1 : $clog2(LED_COUNT);

localparam logic [LED_COUNT-1:0] INITIAL_LED_PATTERN =
    {1'b1, {(LED_COUNT-2){1'b0}}, 1'b1};

typedef enum logic [1:0] {
    STATE_SHIFT_RISE,
    STATE_SHIFT_FALL,
    STATE_LATCH,
    STATE_WAIT
} state_t;

state_t state;
logic [LED_COUNT-1:0] led_pattern;
logic [LED_COUNT-1:0] shift_pattern;
logic [STEP_COUNT_WIDTH-1:0] step_count;
logic [SHIFT_COUNT_WIDTH-1:0] shift_count;
logic [LED_INDEX_WIDTH-1:0] shift_index;
logic stcp;
logic shcp;
logic ser;
logic oen;

// The board is one serpentine chain arranged as two 64-LED rows:
//   serial bits   0..63  -> first row, physically left to right
//   serial bits 64..127 -> second row, physically right to left
// User LED n therefore drives physical serial bits n and 127-n together.
// Rotate the first half upward and the second half downward so both physical
// rows display the same user number while the user sequence remains 0..63.
wire [ROW_LED_COUNT-1:0] next_first_row_pattern =
    {led_pattern[ROW_LED_COUNT-2:0], led_pattern[ROW_LED_COUNT-1]};
wire [ROW_LED_COUNT-1:0] next_second_row_pattern =
    {led_pattern[ROW_LED_COUNT],
     led_pattern[LED_COUNT-1:ROW_LED_COUNT+1]};
wire [LED_COUNT-1:0] next_led_pattern =
    {next_second_row_pattern, next_first_row_pattern};
wire [LED_COUNT-1:0] next_shift_pattern =
    LED_ACTIVE_LOW ? ~next_led_pattern : next_led_pattern;

assign hc595_led.stcp = stcp;
assign hc595_led.shcp = shcp;
assign hc595_led.ser  = ser;
assign hc595_led.oen  = oen;

// Shift the highest bit first and bit 0 last. After LED_COUNT clocks,
// pattern bit 0 is presented on Q0 of the first 74HC595 in the chain.
always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state         <= STATE_SHIFT_RISE;
        led_pattern   <= INITIAL_LED_PATTERN;
        shift_pattern <= LED_ACTIVE_LOW ?
                         ~INITIAL_LED_PATTERN : INITIAL_LED_PATTERN;
        step_count    <= '0;
        shift_count   <= '0;
        shift_index   <= LED_INDEX_WIDTH'(LED_COUNT - 1);
        stcp          <= 1'b0;
        shcp          <= 1'b0;
        ser           <= LED_ACTIVE_LOW ?
                         ~INITIAL_LED_PATTERN[LED_COUNT-1] :
                          INITIAL_LED_PATTERN[LED_COUNT-1];
        // 74HC595 OE is active low. Keep all outputs disabled until the
        // first complete LED_COUNT-bit pattern has been shifted and latched.
        oen           <= 1'b1;
    end else begin
        // STCP is normally low and is held high for one shift half-period.
        stcp <= 1'b0;

        case (state)
            STATE_SHIFT_RISE: begin
                if (shift_count == SHIFT_HALF_CYCLES - 1) begin
                    shift_count <= '0;
                    shcp        <= 1'b1;
                    state       <= STATE_SHIFT_FALL;
                end else begin
                    shift_count <= shift_count + 1'b1;
                end
            end

            STATE_SHIFT_FALL: begin
                if (shift_count == SHIFT_HALF_CYCLES - 1) begin
                    shift_count <= '0;
                    shcp        <= 1'b0;

                    if (shift_index == 0) begin
                        state <= STATE_LATCH;
                    end else begin
                        shift_index <= shift_index - 1'b1;
                        ser <= shift_pattern[shift_index - 1'b1];
                        state <= STATE_SHIFT_RISE;
                    end
                end else begin
                    shift_count <= shift_count + 1'b1;
                end
            end

            STATE_LATCH: begin
                // Update all LED_COUNT parallel outputs simultaneously.
                stcp <= 1'b1;

                if (shift_count == SHIFT_HALF_CYCLES - 1) begin
                    shift_count <= '0;
                    oen         <= 1'b0;
                    step_count  <= '0;
                    state       <= STATE_WAIT;
                end else begin
                    shift_count <= shift_count + 1'b1;
                end
            end

            STATE_WAIT: begin
                if (step_count == STEP_CYCLES - 1) begin
                    step_count    <= '0;
                    led_pattern   <= next_led_pattern;
                    shift_pattern <= next_shift_pattern;
                    shift_index   <= LED_INDEX_WIDTH'(LED_COUNT - 1);
                    shift_count   <= '0;
                    ser           <= next_shift_pattern[LED_COUNT-1];
                    state         <= STATE_SHIFT_RISE;
                end else begin
                    step_count <= step_count + 1'b1;
                end
            end

            default: begin
                state <= STATE_SHIFT_RISE;
                oen   <= 1'b1;
            end
        endcase
    end
end

endmodule
