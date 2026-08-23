`include "hc595.svh"

module HC595LED #(
    parameter integer CHIP_NUMBERS = 16,
    parameter integer CLK_FREQ_HZ = 50_000_000,
    parameter integer SHIFT_CLK_HZ = 1_000_000,
    // 0: a '1' turns an LED on; 1: a '0' turns an LED on.
    parameter bit LED_ACTIVE_LOW = 1'b0
)(
    input  logic        clk,
    input  logic        rstn,
    input  logic [63:0] valve_open_status,
    hc595.master        hc595_led
);

localparam integer USER_LED_COUNT = 64;
localparam integer LED_COUNT = CHIP_NUMBERS * 8;
localparam integer SHIFT_HALF_CYCLES_CALC =
    CLK_FREQ_HZ / (2 * SHIFT_CLK_HZ);
localparam integer SHIFT_HALF_CYCLES =
    (SHIFT_HALF_CYCLES_CALC < 1) ? 1 : SHIFT_HALF_CYCLES_CALC;
localparam integer SHIFT_COUNT_WIDTH =
    (SHIFT_HALF_CYCLES <= 1) ? 1 : $clog2(SHIFT_HALF_CYCLES);
localparam integer LED_INDEX_WIDTH =
    (LED_COUNT <= 1) ? 1 : $clog2(LED_COUNT);
localparam logic [LED_COUNT-1:0] OUTPUTS_OFF =
    LED_ACTIVE_LOW ? {LED_COUNT{1'b1}} : {LED_COUNT{1'b0}};

typedef enum logic [1:0] {
    STATE_SHIFT_RISE,
    STATE_SHIFT_FALL,
    STATE_LATCH,
    STATE_WAIT
} state_t;

state_t state;
logic [LED_COUNT-1:0] shift_pattern;
logic [LED_COUNT-1:0] latched_pattern;
logic [SHIFT_COUNT_WIDTH-1:0] shift_count;
logic [LED_INDEX_WIDTH-1:0] shift_index;
logic stcp;
logic shcp;
logic ser;
logic oen;

function automatic [USER_LED_COUNT-1:0] reverse_user_bits(
    input logic [USER_LED_COUNT-1:0] user_bits
);
    integer bit_index;
    begin
        for (bit_index = 0;
             bit_index < USER_LED_COUNT;
             bit_index = bit_index + 1) begin
            reverse_user_bits[USER_LED_COUNT-1-bit_index] =
                user_bits[bit_index];
        end
    end
endfunction

// The first physical row is serial bits 0..63 from left to right. The second
// row is serial bits 64..127 from right to left, so its bitmap is reversed.
// Consequently each user valve bit lights two LEDs at the matching horizontal
// position: physical serial bits n and 127-n.
wire [LED_COUNT-1:0] logical_led_pattern = {
    reverse_user_bits(valve_open_status),
    valve_open_status
};
wire [LED_COUNT-1:0] desired_shift_pattern =
    LED_ACTIVE_LOW ? ~logical_led_pattern : logical_led_pattern;

assign hc595_led.stcp = stcp;
assign hc595_led.shcp = shcp;
assign hc595_led.ser  = ser;
assign hc595_led.oen  = oen;

// Shift the highest bit first and bit 0 last. After LED_COUNT clocks,
// pattern bit 0 is presented on Q0 of the first 74HC595 in the chain.
always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        state           <= STATE_SHIFT_RISE;
        shift_pattern   <= OUTPUTS_OFF;
        latched_pattern <= OUTPUTS_OFF;
        shift_count     <= '0;
        shift_index     <= LED_INDEX_WIDTH'(LED_COUNT - 1);
        stcp            <= 1'b0;
        shcp            <= 1'b0;
        ser             <= OUTPUTS_OFF[LED_COUNT-1];
        // 74HC595 OE is active low. Keep outputs disabled until the first
        // complete all-off pattern has been shifted and latched.
        oen             <= 1'b1;
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
                // Update all 128 physical LED outputs simultaneously.
                stcp <= 1'b1;

                if (shift_count == SHIFT_HALF_CYCLES - 1) begin
                    shift_count     <= '0;
                    latched_pattern <= shift_pattern;
                    oen             <= 1'b0;
                    state           <= STATE_WAIT;
                end else begin
                    shift_count <= shift_count + 1'b1;
                end
            end

            STATE_WAIT: begin
                // Transfer only when a valve state changes. If another state
                // changes during transfer, it is picked up on the next pass.
                if (desired_shift_pattern != latched_pattern) begin
                    shift_pattern <= desired_shift_pattern;
                    shift_index   <= LED_INDEX_WIDTH'(LED_COUNT - 1);
                    shift_count   <= '0;
                    ser           <= desired_shift_pattern[LED_COUNT-1];
                    state         <= STATE_SHIFT_RISE;
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
