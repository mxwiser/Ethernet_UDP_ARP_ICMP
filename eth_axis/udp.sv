`include "axis.svh"
`include "pc_head.svh"


module udp(
	input	wire						sys_rst_n,
	input   wire                        sys_clk,


	// phy  port
	axis							    m_phy_rx,
	axis							    s_phy_tx,
	input								rx_clk,
	input								tx_clk,
	// user port
	output	wire						udp_rxstart,
	output	wire						udp_rxend,       // not include paddding
	output	wire						udp_rxframe_done,// when padding fifo out.  
	output	wire						udp_rxdv,
	output	wire	[7:0]				udp_rxdata,
	output	wire	[15:0]				udp_rxamount,				// total amount of data, including all pieces
	output	wire	[15:0]				udp_rxnum,					// the order of the received data in this package
	pc_head.master						udp_rx_head,

	//txstart开始前需要确保pc_mac_addr，pc_ip_addr，pc_port，board_port稳定.
	//tx比rx快时rx可能在txstart开始前会覆写mac ，要么在rxstart时把pc信息当作fifo储存下来.
	input	wire						udp_txstart,
	input	wire	[15:0]				udp_txamount,
	input	wire	[7:0]				udp_txdata,
	output	wire						udp_txreq,					// acknowledge that udp_txdata has been transfered
	output	wire						udp_txbusy,
	pc_head.slave						udp_tx_head
);









	axis								rx_net_cdc();	
	axis								tx_sys();					// udp_axis_rx ARP reply
	axis								tx_udp();					// udp_axis_tx UDP data

	
	parameter		BOARD_MAC_ADDR			= 48'h50_12_22_33_44_55;
	parameter		BOARD_IP_ADDR			= 32'h0A_0A_01_0A;			// 10.10.1.10






rx_cdc_fifo_axis u_rx_cdc_fifo_axis(
	.rstn     (sys_rst_n),
	.clk  	  (sys_clk),
	.rx_clk   (rx_clk),
	.s_rx     (m_phy_rx),
	.m_rx     (rx_net_cdc)
);

udp_axis_rx#(
	.BOARD_IP_ADDR   (BOARD_IP_ADDR),
	.BOARD_MAC_ADDR  (BOARD_MAC_ADDR)
)
										u_udp_axis_rx (
	.sys_clk							( sys_clk			),
	.sys_rst_n							( sys_rst_n			),
	.s_axis_rx							( rx_net_cdc		),
	.m_axis_tx							( tx_sys			),
	.udp_rxstart						( udp_rxstart		),
	.udp_rxend							( udp_rxend			),
	.udp_rxframe_done					( udp_rxframe_done	),
	.udp_rxdv							( udp_rxdv			),
	.udp_rxdata							( udp_rxdata		),
	.udp_rxamount						( udp_rxamount		),
	.udp_rxnum							( udp_rxnum			),
	.rx_head							( udp_rx_head)
);


udp_axis_tx#(	
	.BOARD_IP_ADDR   (BOARD_IP_ADDR),
	.BOARD_MAC_ADDR  (BOARD_MAC_ADDR)
)								
										u_udp_axis_tx (
	.sys_clk							( sys_clk			),
	.sys_rst_n							( sys_rst_n			),
	.m_axis_tx							( tx_udp			),
	.udp_txstart						( udp_txstart		),
	.udp_txamount						( udp_txamount		),
	.udp_txdata							( udp_txdata		),
	.udp_txreq							( udp_txreq			),
	.udp_txbusy							( udp_txbusy		),
	.tx_head							( udp_tx_head			)
);



tx_cdc_fifo_axis u_tx_cdc_fifo_axis(
	.clk	 (sys_clk),
	.tx_clk  (tx_clk),
	.rstn	 (sys_rst_n),
	.sys_tx  (tx_sys),
	.udp_tx	 (tx_udp),
	.phy_tx  (s_phy_tx)
);



endmodule
