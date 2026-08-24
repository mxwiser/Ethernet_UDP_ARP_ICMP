`include "axis.svh"
`include "pc_head.svh"
`include "hc595.svh"
// EP4CE10 + IP101GRI UDP 回环（RMII 版本）
// PC 发来的 UDP 数据经 eth_axis 解析后存入 FIFO, 回环发回 PC
// Generate the PLL's active-high power-on reset entirely from the raw input
// clock. This avoids depending on a PLL output clock to release the PLL itself.
module pll_areset_generator #(
	parameter integer INPUT_CLK_FREQ_HZ = 40_000_000,
	parameter integer RESET_TIME_MS = 1
)(
	input  logic clkin,
	output logic areset = 1'b1
);
	localparam integer RESET_CYCLES_CALC =
		(INPUT_CLK_FREQ_HZ / 1_000) * RESET_TIME_MS;
	localparam integer RESET_CYCLES =
		(RESET_CYCLES_CALC < 1) ? 1 : RESET_CYCLES_CALC;
	localparam integer RESET_COUNT_WIDTH =
		(RESET_CYCLES <= 1) ? 1 : $clog2(RESET_CYCLES);

	logic [RESET_COUNT_WIDTH-1:0] reset_count = '0;

	always_ff @(posedge clkin) begin
		if (reset_count == RESET_COUNT_WIDTH'(RESET_CYCLES - 1)) begin
			areset <= 1'b0;
		end else begin
			reset_count <= reset_count + 1'b1;
			areset      <= 1'b1;
		end
	end
endmodule

module top (
	output  logic                       led,
	input	logic                       clkin,
	output  logic                       mdc,
	inout   wire                        mdio,
	input	logic						rmii_clk,
	input	logic					   	rmii_rxdv,
	input	logic	[1:0]				rmii_rxdata,
	output	logic						rmii_txen,
	output	logic	[1:0]				rmii_txdata,
	output	logic						rmii_rst,
	hc595.master						hc595_s1,
	hc595.master						hc595_s2,
	hc595.master						hc595_led,
	input   logic   [3:0] 				addr						
);
	// Single RTL source of truth for the PLL c0 system-clock frequency.
	// When PLL c0 is changed, update this value to match it.
	localparam integer PLL_INPUT_CLK_FREQ_HZ = 40_000_000;
	localparam integer PLL_ARESET_MS = 1;
	localparam integer SYS_CLK_FREQ_HZ = 50_000_000;
	localparam integer POWER_ON_RESET_MS = 11;
	localparam integer POWER_ON_RESET_CYCLES =
		(SYS_CLK_FREQ_HZ / 1_000) * POWER_ON_RESET_MS;
	localparam integer POWER_ON_RESET_COUNT_WIDTH =
		$clog2(POWER_ON_RESET_CYCLES + 1);

	wire clk;
	logic areset_sig;

	pll_areset_generator #(
		.INPUT_CLK_FREQ_HZ ( PLL_INPUT_CLK_FREQ_HZ ),
		.RESET_TIME_MS     ( PLL_ARESET_MS )
	) u_pll_areset_generator (
		.clkin  ( clkin      ),
		.areset ( areset_sig )
	);

	pll	pll_inst (
		.areset ( areset_sig ),
		.inclk0 ( clkin ),
		.c0 ( clk ),
		.c1 ( mdc )
	);



	// L144 has no external reset input. Hold power-on reset for 11 ms;
	// the counter length follows SYS_CLK_FREQ_HZ automatically.
	logic [POWER_ON_RESET_COUNT_WIDTH-1:0] power_on_reset_count = '0;
	wire rstn =
		(power_on_reset_count ==
		 POWER_ON_RESET_COUNT_WIDTH'(POWER_ON_RESET_CYCLES - 1));

	always_ff @(posedge clk) begin
		if (!rstn)
			power_on_reset_count <= power_on_reset_count + 1'b1;
	end

	logic phy_ready;
	logic phy_full_duplex;
	logic rmii_rst_unused;

	phy_smi_helper u_phy_smi_helper (
		.clk     ( clk      ),
		.rst     ( rstn     ),
		.mdclk   ( mdc      ),
		.phyrst  ( rmii_rst ),
		.phy_rdy ( phy_ready),
		.phy_full_duplex ( phy_full_duplex ),
		.mdio    ( mdio     )
	);

	// 板载 LED 低电平点亮：链路就绪且为全双工时点亮。
	assign led = ~(phy_ready & phy_full_duplex);

	wire								udp_rxstart;
	wire								udp_rxend;
	wire								udp_rxframe_done;
	wire								udp_rxdv;
	wire	[7:0]						udp_rxdata;
	wire	[15:0]						udp_rxamount;
	wire	[15:0]						udp_rxnum;
	wire								udp_txstart;
	wire	[15:0]						udp_txamount;
	wire	[7:0]						udp_txdata;
	wire								udp_txreq;
	wire								udp_txbusy;

    axis								m_phy_rx();
	axis								s_phy_tx();
	pc_head							udp_rx_head();
	pc_head							udp_tx_head();
	logic	[47:0]					board_mac_addr;
	logic	[31:0]					board_ip_addr;

	phy_rmii_axis							u_phy_rmii_axis (
		.rstn								( rstn		),
		.rmii_clk							( rmii_clk			),
		.rmii_crs_dv						( rmii_rxdv			),
		.rmii_rxdata						( rmii_rxdata		),
		.rmii_txen							( rmii_txen			),
		.rmii_txdata						( rmii_txdata		),
		.rmii_rst							( rmii_rst_unused	),
		.m_rmii_rx_axis_net					( m_phy_rx			),
		.s_rmii_tx_axis_net					( s_phy_tx     		)
	);

	ip_conf u_ip_conf (
		.clk								( clk			),
		.rstn						    	( rstn			),
		.addr								( addr			),
		.board_mac_addr						( board_mac_addr	),
		.board_ip_addr						( board_ip_addr	)
	);

	udp	u1_udp (
		.sys_rst_n							( rstn		    ),
		.sys_clk							( clk			),
		.board_mac_addr						( board_mac_addr	),
		.board_ip_addr						( board_ip_addr	),
		.m_phy_rx							( m_phy_rx      ),
		.s_phy_tx                           ( s_phy_tx      ),
		.tx_clk								( rmii_clk		),
		.rx_clk								( rmii_clk      ),

		.udp_rxstart						( udp_rxstart	),
		.udp_rxend							( udp_rxend		),
		.udp_rxframe_done					( udp_rxframe_done),
		.udp_rxdv							( udp_rxdv		),
		.udp_rxdata							( udp_rxdata	),
		.udp_rxamount						( udp_rxamount	),//total
		.udp_rxnum							( udp_rxnum		),//count
		.udp_rx_head						( udp_rx_head	),

		.udp_txstart						( udp_txstart	),
		.udp_txamount						( udp_txamount	),
		.udp_txdata							( udp_txdata	),
		.udp_txreq							( udp_txreq		),
		.udp_txbusy							( udp_txbusy	),
		.udp_tx_head						( udp_tx_head	)
	);



	udp_ring u_udp_ring (
		.clk								( clk				),
		.rstn								( rstn				),
		.udp_rxstart						( udp_rxstart		),
		.udp_rxframe_done					( udp_rxframe_done	),
		.udp_rxdv							( udp_rxdv			),
		.udp_rxdata							( udp_rxdata		),
		.udp_rxamount						( udp_rxamount		),
		.udp_rx_head						( udp_rx_head		),
		.udp_txstart						( udp_txstart		),
		.udp_txamount						( udp_txamount		),
		.udp_txdata							( udp_txdata		),
		.udp_txreq							( udp_txreq			),
		.udp_txbusy							( udp_txbusy		),
		.udp_tx_head						( udp_tx_head		)
	);

	udp_cmd_process #(
		.SYS_CLK_FREQ_HZ					( SYS_CLK_FREQ_HZ )
	) u1_udp_cmd_process(
		.clk								( clk				),
		.rstn								( rstn				),
		.udp_rxstart						( udp_rxstart		),
		.udp_rxend							( udp_rxend			),
		.udp_rxframe_done					( udp_rxframe_done	),
		.udp_rxdv							( udp_rxdv			),
		.udp_rxdata							( udp_rxdata			),
		.udp_rxamount						( udp_rxamount		),
		.udp_rx_head						( udp_rx_head		),
		.hc595_s1							( hc595_s1			),
		.hc595_s2							( hc595_s2			),
		.hc595_led							( hc595_led			)
	);



endmodule
