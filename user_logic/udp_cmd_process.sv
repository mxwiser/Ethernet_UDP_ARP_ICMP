`include "hc595.svh"
module udp_cmd_process #(
    parameter integer SYS_CLK_FREQ_HZ = 50_000_000
)(
    input  logic                    clk,
    input  logic                    rstn,
    input  wire                     udp_rxstart,
    input  wire                     udp_rxend,
    input  wire						udp_rxframe_done,
	input  wire						udp_rxdv,
	input  wire [7:0]				udp_rxdata,
	input  wire [15:0]				udp_rxamount,
	pc_head.slave					udp_rx_head,
    hc595                           hc595_s1,
    hc595                           hc595_s2,
    hc595                           hc595_led
);

wire        parser_command_valid;
wire [63:0] parser_command_data;
wire        command_fifo_empty;
wire        command_fifo_full;
wire [63:0] command_fifo_data;
wire        command_fifo_read;
wire        controller_command_ready;
wire        scheduler_reset_pulse;

wire        pwm_s1_wr_en;
wire [5:0]  pwm_s1_wr_addr;
wire [3:0]  pwm_s1_wr_duty;
wire        pwm_s2_wr_en;
wire [5:0]  pwm_s2_wr_addr;
wire [3:0]  pwm_s2_wr_duty;
wire [63:0] valve_open_status;

// Reset is decoded on an independent lane so it cannot be hidden behind an
// OPEN command when the scheduler or the normal command FIFO is full.
udp_scheduler_reset_detector u_udp_scheduler_reset_detector (
    .clk              (clk),
    .rstn             (rstn),
    .udp_rxstart      (udp_rxstart),
    .udp_rxend        (udp_rxend),
    .udp_rxframe_done (udp_rxframe_done),
    .udp_rxdv         (udp_rxdv),
    .udp_rxdata       (udp_rxdata),
    .udp_rxamount     (udp_rxamount),
    .reset_pulse      (scheduler_reset_pulse)
);

// Function 0x01 packets are validated atomically, then their 1..100 command
// records are streamed into a FIFO large enough to hold one complete packet.
udp_command_parser #(
    .MAX_COMMANDS (100)
) u_udp_command_parser (
    .clk              (clk),
    .rstn             (rstn),
    .clear            (scheduler_reset_pulse),
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
    .clear   (scheduler_reset_pulse),
    .wr_en   (parser_command_valid),
    .wr_data (parser_command_data),
    .rd_en   (command_fifo_read),
    .rd_data (command_fifo_data),
    .empty   (command_fifo_empty),
    .full    (command_fifo_full)
);

assign command_fifo_read = !command_fifo_empty && controller_command_ready;

valve_controller #(
    .CLK_FREQ_HZ (SYS_CLK_FREQ_HZ),
    .TIMER_HZ    (10_000),
    .VALVE_COUNT (64),
    .PWM_LEVELS  (10),
    .SCHEDULE_DEPTH (1024)
) u_valve_controller (
    .clk              (clk),
    .rstn             (rstn),
    .scheduler_reset  (scheduler_reset_pulse),
    .command_valid    (command_fifo_read),
    .command_data     (command_fifo_data),
    .command_ready    (controller_command_ready),
    .pwm_s1_wr_en     (pwm_s1_wr_en),
    .pwm_s1_wr_addr   (pwm_s1_wr_addr),
    .pwm_s1_wr_duty   (pwm_s1_wr_duty),
    .pwm_s2_wr_en     (pwm_s2_wr_en),
    .pwm_s2_wr_addr   (pwm_s2_wr_addr),
    .pwm_s2_wr_duty   (pwm_s2_wr_duty),
    .valve_open_status(valve_open_status)
);

HC595PWM #(
    .CHIP_NUMBERS (8),
    .CLK_FREQ_HZ  (SYS_CLK_FREQ_HZ),
    .PWM_FREQ_HZ  (1_000),
    .PWM_LEVELS   (10)
) u1_hc595 (
    .clk           (clk),
    .rstn          (rstn),
    .pwm_wr_en     (pwm_s1_wr_en),
    .pwm_wr_addr   (pwm_s1_wr_addr),
    .pwm_wr_duty   (pwm_s1_wr_duty),
    .init_done     (),
    .hc595_serial (hc595_s1)
);

HC595PWM #(
    .CHIP_NUMBERS (8),
    .CLK_FREQ_HZ  (SYS_CLK_FREQ_HZ),
    .PWM_FREQ_HZ  (1_000),
    .PWM_LEVELS   (10)
) u2_hc595 (
    .clk           (clk),
    .rstn          (rstn),
    .pwm_wr_en     (pwm_s2_wr_en),
    .pwm_wr_addr   (pwm_s2_wr_addr),
    .pwm_wr_duty   (pwm_s2_wr_duty),
    .init_done     (),
    .hc595_serial (hc595_s2)
);

HC595LED #(
    .CHIP_NUMBERS (16),
    .CLK_FREQ_HZ  (SYS_CLK_FREQ_HZ),
    .SHIFT_CLK_HZ (1_000_000),
    .REFRESH_HZ   (1_000),
    .LED_ACTIVE_LOW (1'b0)
) u_hc595_led (
    .clk       (clk),
    .rstn      (rstn),
    .valve_open_status (valve_open_status),
    .hc595_led (hc595_led)
);
endmodule
