`include "axis.svh"
`include "pc_head.svh"

// EP4CE10 + LAN8720A UDP 回环（RMII 版本）
// PC 发来的 UDP 数据经 eth_axis 解析后存入 FIFO, 回环发回 PC
module top (
	output  logic                       uart_txd,
	input   logic                       uart_rxd,
	input	logic						rstn,
	output  logic	[1:0]				led,
	input	logic                       clk,
	input	logic						rmii_clk,
	input	logic					   	rmii_rxdv,
	input	logic	[1:0]				rmii_rxdata,
	output	logic						rmii_txen,
	output	logic	[1:0]				rmii_txdata,
	output	logic						rmii_rst
);

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

phy_rmii_axis							u_phy_rmii_axis (
	.rstn								( rstn		),
	.rmii_clk							( rmii_clk			),
	.rmii_crs_dv						( rmii_rxdv			),
	.rmii_rxdata						( rmii_rxdata		),
	.rmii_txen							( rmii_txen			),
	.rmii_txdata						( rmii_txdata		),
	.rmii_rst							( rmii_rst			),
	.m_rmii_rx_axis_net					( m_phy_rx			),
	.s_rmii_tx_axis_net					( s_phy_tx     		)
);

udp	u1_udp (
	.sys_rst_n							( rstn		    ),
	.sys_clk							( clk			),
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
	.udp_rxdata							( udp_rxdata			),
	.udp_rxamount						( udp_rxamount		),
	.udp_rx_head						( udp_rx_head		),
	.udp_txstart							( udp_txstart		),
	.udp_txamount						( udp_txamount		),
	.udp_txdata							( udp_txdata			),
	.udp_txreq							( udp_txreq			),
	.udp_txbusy							( udp_txbusy			),
	.udp_tx_head						( udp_tx_head		)
);

// user test
always @(posedge clk or negedge rstn) begin
	if (!rstn) begin
		led <= 2'b0;
	end else if (udp_rxdv && (udp_rxdata == 8'hA1)) begin
		led <= ~led;
	end
end

endmodule
