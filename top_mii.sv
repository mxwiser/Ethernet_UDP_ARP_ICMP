`include "axis.svh"
`include "pc_head.svh"


// PC 发来的 UDP 数据经 eth_axis 解析后存入 FIFO, 回环发回 PC
module top_mii (
	output  logic                       uart_txd,
	input   logic                       uart_rxd,
	input	logic						rstn,
	output  logic	[1:0]				led,
	input	logic                       clk,

	//smi   
	output  logic  						phy_rst,
	output  wire                        mdc,
	inout   wire						mdio,
	//MII
	input   logic   					mii_rxdv,
	input   logic	[3:0]				mii_rxd,
	input	logic						mii_rxc,
	output	logic						mii_txen,
	output  logic	[3:0]				mii_txd,
	input   logic   					mii_txc

);
	logic phy_rdy;
	logic phy_full_duplex;

	pll	pll_inst (
		.inclk0 ( clk ),
		.c0     ( mdc )
	);

	phy_smi_helper u_phy_smi_helper (
		.clk								( clk		),
		.rst								( rstn		),
		.mdclk							    ( mdc		),
		.phyrst							    ( phy_rst	),
		.phy_rdy							( phy_rdy	),
		.phy_full_duplex					( phy_full_duplex ),
		.mdio								( mdio		)
	);

	assign led[0] = phy_rdy&phy_full_duplex;          
	

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

phy_mii_axis							u_phy_mii_axis (
	.rstn								( rstn		),
	.tx_clk								( mii_txc	),
	.txd								( mii_txd	),
	.txen								( mii_txen	),
	.rx_clk								( mii_rxc	),
	.rxd								( mii_rxd	),
	.rxdv								( mii_rxdv	),
	.m_phy_rx							( m_phy_rx	),
	.s_phy_tx							( s_phy_tx	)
);

udp	u1_udp (
	.sys_rst_n							( rstn		    ),
	.sys_clk							( clk			),
	.m_phy_rx							( m_phy_rx      ),
	.s_phy_tx                           ( s_phy_tx      ),
	.tx_clk								( mii_txc		),
	.rx_clk								( mii_rxc		),	

	.udp_rxstart						( udp_rxstart	),
	.udp_rxend							( udp_rxend		),
	.udp_rxframe_done					( udp_rxframe_done),
	.udp_rxdv							( udp_rxdv		),
	.udp_rxdata							( udp_rxdata	),
	.udp_rxamount						( udp_rxamount	),//total
	.udp_rxnum							( udp_rxnum		),//count
	.udp_rx_head						( udp_rx_head   ),

	.udp_txstart						( udp_txstart	),
	.udp_txamount						( udp_txamount	),
	.udp_txdata							( udp_txdata	),
	.udp_txreq							( udp_txreq		),
	.udp_txbusy							( udp_txbusy	),
	.udp_tx_head						( udp_tx_head   )
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

//user test
// always @ ( posedge clk or negedge rstn ) begin
// 	if ( !rstn ) begin
// 		led <= 2'b0;
// 	end else if ( udp_rxdv && ( udp_rxdata == 'hA1 ) ) begin
// 		led <= ~led;
// 	end
// end




endmodule
