`include "axis.svh"

// author:		Benjamin SMith
// create time:	2023/03/17 17:24
// edit time:	2026/08/07
// platform:	Cyclone ep4ce10f17i7, board
// module:		arp_axis
// function:	ARP request receive and response, ICMP echo request receive and echo reply, IPv4 only
//				extracted from udp_axis_rx.sv, AXIS version
//				ICMP echo reply pattern (checksum +0x0800 trick, id/seq/payload echo) from udp_lean.sv
// version:		2.0

module eth_axis (
	input	wire							sys_clk,
	input	wire							sys_rst_n,
	input	wire	[47:0]					board_mac_addr,
	input	wire	[31:0]					board_ip_addr,
	axis.slave								s_axis_rx,				// PHY RX stream from rmii_axis, tlast is frame-level
	axis.master								m_axis_tx,				// ARP / ICMP echo reply TX stream to rmii_axis
	output	reg								arp_working,			// ARP / ICMP reply is being sent, for TX arbitration
	
	output	reg		[47:0]					arp_pc_mac,				// PC MAC learned by ARP
	output	reg		[31:0]					arp_pc_ip,				// PC IP learned by ARP
	output	wire							arp_pc_refresh			// learned pulse
);

// -------------------------------- axis <-> gmii bridge ------------------------------------------
// RX: s_mac_tvalid / s_mac_tlast / s_mac_tdata come from s_axis_rx, tready is ignored (rmii_axis never backpressures RX)
	wire									s_mac_tvalid;
	wire									s_mac_tlast;				// frame-level: high in frame, falling edge = frame end
	wire		[7:0]						s_mac_tdata	;
	assign		s_mac_tlast			=		s_axis_rx.tlast;
	assign		s_mac_tvalid		=		s_axis_rx.tvalid; 
	assign		s_mac_tdata			=		s_axis_rx.tdata;
	assign		s_axis_rx.tready	=	1'b1;

// TX (ARP / ICMP reply only): txen / txbusy mapped to AXIS handshake.
// txen && !txbusy  ==  tvalid && tready
    logic [7:0] arp_head [0:8] = '{8'h08,8'h06,8'h00,8'h01,8'h08,8'h00,8'h06,8'h04,8'h00};
    logic [7:0] eth_head [0:7];
	always_comb begin
		eth_head[0] = board_mac_addr[47:40];
		eth_head[1] = board_mac_addr[39:32];
		eth_head[2] = board_mac_addr[31:24];
		eth_head[3] = board_mac_addr[23:16];
		eth_head[4] = board_mac_addr[15:8];
		eth_head[5] = board_mac_addr[7:0];
		eth_head[6] = 8'h08;
		eth_head[7] = 8'h06;
	end
	wire									txen;
	wire									txbusy;
	wire		[7:0]						txdata;
	assign		txen			=	tx_active;
	assign		txbusy			=	!( txen && m_axis_tx.tready );
	assign		m_axis_tx.tvalid	=	txen;
	assign		m_axis_tx.tlast	=	txen;					// frame-level: high in frame, falling edge = frame end
	assign		m_axis_tx.tuser	=	1'b0;
	assign		m_axis_tx.tkeep	=	1'b1;
	assign		m_axis_tx.tstrb	=	1'b1;
	assign		m_axis_tx.tdata	=	txdata;
// ================================ ARP part (from eth_arp_gmii.v) ================================
	localparam		IDLE					= 28'h0_0001,
					RX_SFD					= 28'h0_0002,						// (0xD5)
					MAC_DES					= 28'h0_0004,
					MAC_SRC					= 28'h0_0008,
					TYPE					= 28'h0_0010,						// MAC package
					ARP_TYPE				= 28'h0_0020,						// ARP_TYPE include hardware type (2B), protocol type (2B), MAC length (1B), IP length (1B), 'h0001_0800_0604
					ARP_OPCODE				= 28'h0_0040,
					ARP_SRC_MAC				= 28'h0_0080,
					ARP_SRC_IP				= 28'h0_0100,
					ARP_DES_MAC				= 28'h0_0200,
					ARP_DES_IP				= 28'h0_0400,
					ARP_FILL				= 28'h0_0800,						// ARP
					CRC						= 28'h0_1000,
					IP_TYPE					= 28'h0_2000,						// IPv4 version + IHL, TOS
					IP_LEN					= 28'h0_4000,						// total length
					IP_ID					= 28'h0_8000,						// identification
					IP_SPLIT				= 28'h1_0000,						// flags + fragment offset
					IP_TTL					= 28'h2_0000,
					IP_PROTOCOL				= 28'h4_0000,						// ICMP: 1
					IP_CHECK				= 28'h8_0000,						// IP header checksum, ignore it
					IP_ADDR					= 28'h10_0000,						// source IP + destination IP
					IP_FILL					= 28'h20_0000,						// when IP header length > 20, filled data
					ICMP_TYPE				= 28'h40_0000,						// ICMP type + code, echo request = 'h0800
					ICMP_CHECK				= 28'h80_0000,						// ICMP checksum
					ICMP_ID					= 28'h100_0000,						// ICMP identifier
					ICMP_SEQ				= 28'h200_0000,						// ICMP sequence
					ICMP_DATA				= 28'h400_0000,						// ICMP payload + padding
					ICMP_CRC				= 28'h800_0000;

// -------------------------------- receive arp request ------------------------------------------
	reg		[7:0]							s_mac_tdata_r;
	reg		[27:0]							rx_state;
	reg		[2:0]							rx_cnt_pre;
	reg		[2:0]							rx_cnt_mac_des;
	reg		[47:0]							mac_des;
	reg		[2:0]							rx_cnt_mac_src;
	reg		[47:0]							mac_src;
	reg										rx_cnt_type;
	reg		[2:0]							rx_cnt_arp_type;
	reg		[47:0]							arp_type;
	reg										rx_cnt_arp_opcode;
	reg		[2:0]							rx_cnt_arp_src_mac;
	reg		[1:0]							rx_cnt_arp_src_ip;
	reg		[31:0]							ip_src;
	reg		[2:0]							rx_cnt_arp_des_mac;
	reg		[2:0]							rx_cnt_arp_des_ip;
	reg		[31:0]							ip_des;
	reg		[4:0]							rx_cnt_arp_fill;
	reg		[2:0]							rx_cnt_crc;
	reg		[31:0]							rx_crc32_read;
	reg										arp_req;						// correct ARP request sign
	reg										arp_req_true;					// ARP request sign after crc32 check
	reg										arp_resp;						// arp response starting signal
	
always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_state <= IDLE;
	end else case (rx_state)
		IDLE: begin
			if ( ( rx_cnt_pre == 3'd6 ) && s_mac_tvalid && ( s_mac_tdata == 8'h55 ) ) begin
				rx_state <= RX_SFD;
			end else begin
				rx_state <= IDLE;
			end
		end
		RX_SFD: begin
			if ( s_mac_tvalid && ( s_mac_tdata == 8'hD5 ) ) begin				// when correct SFD (0xD5) is received, jump to next state
				rx_state <= MAC_DES;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= RX_SFD;
			end
		end
		MAC_DES: begin
			if ( rx_cnt_mac_des >= 3'd6 && ( mac_des == 48'hFF_FF_FF_FF_FF_FF || mac_des == board_mac_addr ) ) begin
				rx_state <= MAC_SRC;
			end else if ( rx_cnt_mac_des >= 3'd6 ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= MAC_DES;
			end
		end
		MAC_SRC: begin
			if ( rx_cnt_mac_src == 3'd5 && s_mac_tvalid ) begin
				rx_state <= TYPE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= MAC_SRC;
			end
		end
		TYPE: begin															// only ARP / IPv4 protocol is supported, TYPE = 'h0806 / 'h0800
			if ( rx_cnt_type && s_mac_tvalid && ( { s_mac_tdata_r, s_mac_tdata } == 16'h0806 ) ) begin
				rx_state <= ARP_TYPE;
			end else if ( rx_cnt_type && s_mac_tvalid && ( { s_mac_tdata_r, s_mac_tdata } == 16'h0800 ) ) begin
				rx_state <= IP_TYPE;
			end else if ( rx_cnt_type && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= TYPE;
			end
		end
		ARP_TYPE: begin														// only IPv4 is supported
			if ( rx_cnt_arp_type >= 3'd6 && arp_type == 48'h0001_0800_0604 ) begin
				rx_state <= ARP_OPCODE;
			end else if ( rx_cnt_arp_type >= 3'd6 ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_TYPE;
			end
		end
		ARP_OPCODE: begin													// 1: request, 2: response, detect request
			if ( rx_cnt_arp_opcode && s_mac_tvalid && ( { s_mac_tdata_r, s_mac_tdata } == 16'h0001 ) ) begin
				rx_state <= ARP_SRC_MAC;
			end else if ( rx_cnt_arp_opcode && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_OPCODE;
			end
		end
		ARP_SRC_MAC: begin													// this information has got in MAC_SRC state. ignore it
			if ( rx_cnt_arp_src_mac >= 3'd5 && s_mac_tvalid ) begin
				rx_state <= ARP_SRC_IP;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_SRC_MAC;
			end
		end
		ARP_SRC_IP: begin
			if ( rx_cnt_arp_src_ip >= 2'd3 && s_mac_tvalid ) begin
				rx_state <= ARP_DES_MAC;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_SRC_IP;
			end
		end
		ARP_DES_MAC: begin
			if ( rx_cnt_arp_des_mac >= 3'd5 && s_mac_tvalid ) begin
				rx_state <= ARP_DES_IP;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_DES_MAC;
			end
		end
		ARP_DES_IP: begin
			if ( rx_cnt_arp_des_ip >= 3'd3 && s_mac_tvalid ) begin
				rx_state <= ARP_FILL;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_DES_IP;
			end
		end
		ARP_FILL: begin
			if ( rx_cnt_arp_fill >= 5'd17 && s_mac_tvalid ) begin
				rx_state <= CRC;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ARP_FILL;
			end
		end
		CRC: begin
			if ( rx_cnt_crc >= 3'd3 && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= CRC;
			end
		end
// ================================ ICMP echo request part ================================
		IP_TYPE: begin														// only IPv4 is supported
			if ( rx_cnt_ip_type && s_mac_tvalid && s_mac_tdata_r[7:4] == 4'h4 ) begin
				rx_state <= IP_LEN;
			end else if ( rx_cnt_ip_type && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_TYPE;
			end
		end
		IP_LEN: begin
			if ( rx_cnt_ip_len && s_mac_tvalid ) begin
				rx_state <= IP_ID;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_LEN;
			end
		end
		IP_ID: begin
			if ( rx_cnt_ip_id && s_mac_tvalid ) begin
				rx_state <= IP_SPLIT;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_ID;
			end
		end
		IP_SPLIT: begin
			if ( rx_cnt_ip_split && s_mac_tvalid ) begin
				rx_state <= IP_TTL;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_SPLIT;
			end
		end
		IP_TTL: begin
			if ( s_mac_tvalid ) begin
				rx_state <= IP_PROTOCOL;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_TTL;
			end
		end
		IP_PROTOCOL: begin													// only ICMP protocol is supported, protocol = 1
			if ( s_mac_tvalid && s_mac_tdata == 8'h01 ) begin
				rx_state <= IP_CHECK;
			end else if ( s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_PROTOCOL;
			end
		end
		IP_CHECK: begin														// IP header checksum, ignore it
			if ( rx_cnt_ip_check && s_mac_tvalid ) begin
				rx_state <= IP_ADDR;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_CHECK;
			end
		end
		IP_ADDR: begin														// source IP (4B) + destination IP (4B)
			if ( rx_cnt_ip_addr >= 3'd7 && s_mac_tvalid && ip_header_len > 6'd20 ) begin
				rx_state <= IP_FILL;
			end else if ( rx_cnt_ip_addr >= 3'd7 && s_mac_tvalid ) begin
				rx_state <= ICMP_TYPE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_ADDR;
			end
		end
		IP_FILL: begin														// skip bytes when IP header length > 20
			if ( rx_cnt_ip_fill >= ( ip_header_len - 6'd21 ) && s_mac_tvalid ) begin
				rx_state <= ICMP_TYPE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= IP_FILL;
			end
		end
		ICMP_TYPE: begin													// only echo request is supported, TYPE = 'h0800
			if ( rx_cnt_icmp_type == 2'd0 && s_mac_tvalid && ip_des != board_ip_addr ) begin	// dst IP is complete now
				rx_state <= IDLE;
			end else if ( rx_cnt_icmp_type == 2'd1 && s_mac_tvalid && ( { s_mac_tdata_r, s_mac_tdata } == 16'h0800 ) ) begin
				rx_state <= ICMP_CHECK;
			end else if ( rx_cnt_icmp_type == 2'd1 && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_TYPE;
			end
		end
		ICMP_CHECK: begin
			if ( rx_cnt_icmp_check && s_mac_tvalid ) begin
				rx_state <= ICMP_ID;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_CHECK;
			end
		end
		ICMP_ID: begin
			if ( rx_cnt_icmp_id && s_mac_tvalid ) begin
				rx_state <= ICMP_SEQ;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_ID;
			end
		end
		ICMP_SEQ: begin
			if ( rx_cnt_icmp_seq == 2'd1 && s_mac_tvalid ) begin
				rx_state <= ICMP_DATA;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_SEQ;
			end
		end
		ICMP_DATA: begin													// payload + padding to minimum frame, CRC covered
			if ( rx_cnt_icmp_data >= icmp_data_cnt_max - 12'd1 && s_mac_tvalid ) begin
				rx_state <= ICMP_CRC;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_DATA;
			end
		end
		ICMP_CRC: begin
			if ( rx_cnt_crc >= 3'd3 && s_mac_tvalid ) begin
				rx_state <= IDLE;
			end else if ( !s_mac_tlast ) begin									// frame ends, back to idle
				rx_state <= IDLE;
			end else begin
				rx_state <= ICMP_CRC;
			end
		end
		default: rx_state <= IDLE;
	endcase
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin						// delay of s_mac_tdata
	if ( !sys_rst_n ) begin
		s_mac_tdata_r <= 8'h0;
	end else if ( s_mac_tvalid ) begin
		s_mac_tdata_r <= s_mac_tdata;
	end else begin
		s_mac_tdata_r <= s_mac_tdata_r;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_pre <= 3'd0;
	end else if ( rx_state == IDLE ) begin									// preamble counter, working in idle state
		if ( s_mac_tvalid && ( s_mac_tdata == 8'h55 ) ) begin					// receive 7 0x55 then jump to SFD
			rx_cnt_pre <= rx_cnt_pre + 3'd1;
		end else if ( s_mac_tvalid ) begin
			rx_cnt_pre <= 3'd0;
		end else begin
			rx_cnt_pre <= rx_cnt_pre;
		end
	end else begin
		rx_cnt_pre <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_mac_des <= 3'd0;
	end else if ( rx_state == MAC_DES ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_mac_des <= rx_cnt_mac_des + 3'd1;
		end else begin
			rx_cnt_mac_des <= rx_cnt_mac_des;
		end
	end else begin
		rx_cnt_mac_des <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		mac_des <= 48'h0;
	end else if ( rx_state == MAC_DES ) begin								// receive 6 byte destination MAC address, when it's broadcast address, jump to next state
		if ( rx_cnt_mac_des == 3'd0 && s_mac_tvalid ) begin
			mac_des <= { s_mac_tdata, mac_des[39:0] };
		end else if ( rx_cnt_mac_des == 3'd1 && s_mac_tvalid ) begin
			mac_des <= { mac_des[47:40], s_mac_tdata, mac_des[31:0] };
		end else if ( rx_cnt_mac_des == 3'd2 && s_mac_tvalid ) begin
			mac_des <= { mac_des[47:32], s_mac_tdata, mac_des[23:0] };
		end else if ( rx_cnt_mac_des == 3'd3 && s_mac_tvalid ) begin
			mac_des <= { mac_des[47:24], s_mac_tdata, mac_des[15:0] };
		end else if ( rx_cnt_mac_des == 3'd4 && s_mac_tvalid ) begin
			mac_des <= { mac_des[47:16], s_mac_tdata, mac_des[7:0] };
		end else if ( rx_cnt_mac_des == 3'd5 && s_mac_tvalid ) begin
			mac_des <= { mac_des[47:8], s_mac_tdata };
		end else begin
			mac_des <= mac_des;
		end
	end else begin
		mac_des <= mac_des;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_mac_src <= 3'd0;
	end else if ( rx_state == MAC_SRC ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_mac_src <= rx_cnt_mac_src + 3'd1;
		end else begin
			rx_cnt_mac_src <= rx_cnt_mac_src;
		end
	end else begin
		rx_cnt_mac_src <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		mac_src <= 48'h0;
	end else if ( rx_state == MAC_SRC ) begin								// receive 6 byte source MAC address, and save it in mac_src temporary
		if ( rx_cnt_mac_src == 3'd0 && s_mac_tvalid ) begin					// when destination IP address is correct, acknowledge this address
			mac_src <= { s_mac_tdata, mac_src[39:0] };
		end else if ( rx_cnt_mac_src == 3'd1 && s_mac_tvalid ) begin
			mac_src <= { mac_src[47:40], s_mac_tdata, mac_src[31:0] };
		end else if ( rx_cnt_mac_src == 3'd2 && s_mac_tvalid ) begin
			mac_src <= { mac_src[47:32], s_mac_tdata, mac_src[23:0] };
		end else if ( rx_cnt_mac_src == 3'd3 && s_mac_tvalid ) begin
			mac_src <= { mac_src[47:24], s_mac_tdata, mac_src[15:0] };
		end else if ( rx_cnt_mac_src == 3'd4 && s_mac_tvalid ) begin
			mac_src <= { mac_src[47:16], s_mac_tdata, mac_src[7:0] };
		end else if ( rx_cnt_mac_src == 3'd5 && s_mac_tvalid ) begin
			mac_src <= { mac_src[47:8], s_mac_tdata };
		end else begin
			mac_src <= mac_src;
		end
	end else begin
		mac_src <= mac_src;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_type <= 1'b0;
	end else if ( rx_state == TYPE ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_type <= ~rx_cnt_type;
		end else begin
			rx_cnt_type <= rx_cnt_type;
		end
	end else begin
		rx_cnt_type <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_type <= 3'd0;
	end else if ( rx_state == ARP_TYPE ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_type <= rx_cnt_arp_type + 3'd1;
		end else begin
			rx_cnt_arp_type <= rx_cnt_arp_type;
		end
	end else begin
		rx_cnt_arp_type <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_type <= 48'h0;
	end else if ( rx_state == ARP_TYPE ) begin
		if ( s_mac_tvalid && rx_cnt_arp_type == 3'd0 ) begin					// hardware type, 'h0001 means Ethernet
			arp_type <= { s_mac_tdata, arp_type[39:0] };
		end else if ( s_mac_tvalid && rx_cnt_arp_type == 3'd1 ) begin
			arp_type <= { arp_type[47:40], s_mac_tdata, arp_type[31:0] };
		end else if ( s_mac_tvalid && rx_cnt_arp_type == 3'd2 ) begin			// protocol type, 'h0800 means IPv4
			arp_type <= { arp_type[47:32], s_mac_tdata, arp_type[23:0] };
		end else if ( s_mac_tvalid && rx_cnt_arp_type == 3'd3 ) begin
			arp_type <= { arp_type[47:24], s_mac_tdata, arp_type[15:0] };
		end else if ( s_mac_tvalid && rx_cnt_arp_type == 3'd4 ) begin			// MAC address length, which must be 6
			arp_type <= { arp_type[47:16], s_mac_tdata, arp_type[7:0] };
		end else if ( s_mac_tvalid && rx_cnt_arp_type == 3'd5 ) begin			// IP address length, which is 4 in IPv4
			arp_type <= { arp_type[47:8], s_mac_tdata };
		end else begin
			arp_type <= arp_type;
		end
	end else begin
		arp_type <= arp_type;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_opcode <= 1'b0;
	end else if ( rx_state == ARP_OPCODE ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_opcode <= ~rx_cnt_arp_opcode;
		end else begin
			rx_cnt_arp_opcode <= rx_cnt_arp_opcode;
		end
	end else begin
		rx_cnt_arp_opcode <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_src_mac <= 3'd0;
	end else if ( rx_state == ARP_SRC_MAC ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_src_mac <= rx_cnt_arp_src_mac + 3'd1;
		end else begin
			rx_cnt_arp_src_mac <= rx_cnt_arp_src_mac;
		end
	end else begin
		rx_cnt_arp_src_mac <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_src_ip <= 2'd0;
	end else if ( rx_state == ARP_SRC_IP ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_src_ip <= rx_cnt_arp_src_ip + 2'd1;
		end else begin
			rx_cnt_arp_src_ip <= rx_cnt_arp_src_ip;
		end
	end else begin
		rx_cnt_arp_src_ip <= 2'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		ip_src <= 32'h0;
	end else if ( rx_state == ARP_SRC_IP ) begin							// receive 4 byte source IP address, and save it in ip_src temporary
		if ( rx_cnt_arp_src_ip == 2'd0 && s_mac_tvalid ) begin					// when destination IP address is correct, acknowledge it
			ip_src <= { s_mac_tdata, ip_src[23:0] };
		end else if ( rx_cnt_arp_src_ip == 2'd1 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:24], s_mac_tdata, ip_src[15:0] };
		end else if ( rx_cnt_arp_src_ip == 2'd2 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:16], s_mac_tdata, ip_src[7:0] };
		end else if ( rx_cnt_arp_src_ip == 2'd3 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:8], s_mac_tdata };
		end else begin
			ip_src <= ip_src;
		end
	end else if ( rx_state == IP_ADDR ) begin								// IPv4: first 4 bytes of source IP
		if ( rx_cnt_ip_addr == 3'd0 && s_mac_tvalid ) begin
			ip_src <= { s_mac_tdata, ip_src[23:0] };
		end else if ( rx_cnt_ip_addr == 3'd1 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:24], s_mac_tdata, ip_src[15:0] };
		end else if ( rx_cnt_ip_addr == 3'd2 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:16], s_mac_tdata, ip_src[7:0] };
		end else if ( rx_cnt_ip_addr == 3'd3 && s_mac_tvalid ) begin
			ip_src <= { ip_src[31:8], s_mac_tdata };
		end else begin
			ip_src <= ip_src;
		end
	end else begin
		ip_src <= ip_src;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_des_mac <= 3'd0;
	end else if ( rx_state == ARP_DES_MAC ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_des_mac <= rx_cnt_arp_des_mac + 3'd1;
		end else begin
			rx_cnt_arp_des_mac <= rx_cnt_arp_des_mac;
		end
	end else begin
		rx_cnt_arp_des_mac <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_des_ip <= 3'd0;
	end else if ( rx_state == ARP_DES_IP ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_des_ip <= rx_cnt_arp_des_ip + 3'd1;
		end else begin
			rx_cnt_arp_des_ip <= rx_cnt_arp_des_ip;
		end
	end else begin
		rx_cnt_arp_des_ip <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		ip_des <= 32'h0;
	end else if ( rx_state == ARP_DES_IP ) begin							// read destination IP
		if ( rx_cnt_arp_des_ip == 3'd0 && s_mac_tvalid ) begin
			ip_des <= { s_mac_tdata, ip_des[23:0] };
		end else if ( rx_cnt_arp_des_ip == 3'd1 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:24], s_mac_tdata, ip_des[15:0] };
		end else if ( rx_cnt_arp_des_ip == 3'd2 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:16], s_mac_tdata, ip_des[7:0] };
		end else if ( rx_cnt_arp_des_ip == 3'd3 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:8], s_mac_tdata };
		end else begin
			ip_des <= ip_des;
		end
	end else if ( rx_state == IP_ADDR ) begin								// IPv4: last 4 bytes of destination IP
		if ( rx_cnt_ip_addr == 3'd4 && s_mac_tvalid ) begin
			ip_des <= { s_mac_tdata, ip_des[23:0] };
		end else if ( rx_cnt_ip_addr == 3'd5 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:24], s_mac_tdata, ip_des[15:0] };
		end else if ( rx_cnt_ip_addr == 3'd6 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:16], s_mac_tdata, ip_des[7:0] };
		end else if ( rx_cnt_ip_addr == 3'd7 && s_mac_tvalid ) begin
			ip_des <= { ip_des[31:8], s_mac_tdata };
		end else begin
			ip_des <= ip_des;
		end
	end else begin
		ip_des <= ip_des;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_arp_fill <= 5'd0;
	end else if ( rx_state == ARP_FILL ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_arp_fill <= rx_cnt_arp_fill + 5'd1;
		end else begin
			rx_cnt_arp_fill <= rx_cnt_arp_fill;
		end
	end else begin
		rx_cnt_arp_fill <= 5'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_crc <= 3'd0;
	end else if ( rx_state == CRC || rx_state == ICMP_CRC ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_crc <= rx_cnt_crc + 3'd1;
		end else begin
			rx_cnt_crc <= rx_cnt_crc;
		end
	end else begin
		rx_cnt_crc <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_crc32_read <= 32'h0;
	end else if ( rx_state == CRC || rx_state == ICMP_CRC ) begin
		if ( rx_cnt_crc == 3'd0 && s_mac_tvalid ) begin
			rx_crc32_read <= { rx_crc32_read[31:8], s_mac_tdata };
		end else if ( rx_cnt_crc == 3'd1 && s_mac_tvalid ) begin
			rx_crc32_read <= { rx_crc32_read[31:16], s_mac_tdata, rx_crc32_read[7:0] };
		end else if ( rx_cnt_crc == 3'd2 && s_mac_tvalid ) begin
			rx_crc32_read <= { rx_crc32_read[31:24], s_mac_tdata, rx_crc32_read[15:0] };
		end else if ( rx_cnt_crc == 3'd3 && s_mac_tvalid ) begin
			rx_crc32_read <= { s_mac_tdata, rx_crc32_read[23:0] };
		end else begin
			rx_crc32_read <= rx_crc32_read;
		end
	end else begin
		rx_crc32_read <= rx_crc32_read;
	end
end

// ================================ ICMP echo request receive ================================
	reg										rx_cnt_ip_type;
	reg		[5:0]							ip_header_len;					// IP header length in bytes (IHL * 4)
	reg										rx_cnt_ip_len;
	reg		[15:0]							ip_total_len;					// IP total length from header
	reg										rx_cnt_ip_id;
	reg		[15:0]							ip_id;							// IP identification
	reg										rx_cnt_ip_split;
	reg										rx_cnt_ip_check;
	reg		[2:0]							rx_cnt_ip_addr;
	reg		[5:0]							rx_cnt_ip_fill;
	reg		[1:0]							rx_cnt_icmp_type;
	reg										rx_cnt_icmp_check;
	reg		[15:0]							icmp_checksum;					// ICMP checksum from request, reply = it + 0x0800
	reg										rx_cnt_icmp_id;
	reg		[15:0]							icmp_id;						// ICMP identifier, echoed in reply
	reg		[1:0]							rx_cnt_icmp_seq;
	reg		[15:0]							icmp_seq;						// ICMP sequence, echoed in reply
	reg		[11:0]							rx_cnt_icmp_data;
	reg		[15:0]							icmp_payload_len;				// payload bytes to echo = ip_total_len - header - 8
	wire	[11:0]							icmp_data_cnt_max				= ( icmp_payload_len >= 16'd18 ) ? icmp_payload_len[11:0] : 12'd18;	// payload + padding to 60B frame

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_type <= 1'b0;
	end else if ( rx_state == IP_TYPE ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_type <= ~rx_cnt_ip_type;
		end else begin
			rx_cnt_ip_type <= rx_cnt_ip_type;
		end
	end else begin
		rx_cnt_ip_type <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		ip_header_len <= 6'd0;
	end else if ( rx_state == IP_TYPE && !rx_cnt_ip_type && s_mac_tvalid ) begin
		ip_header_len <= { 2'b00, s_mac_tdata[3:0] } << 2;				// IHL * 4 bytes
	end else begin
		ip_header_len <= ip_header_len;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_len <= 1'b0;
	end else if ( rx_state == IP_LEN ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_len <= ~rx_cnt_ip_len;
		end else begin
			rx_cnt_ip_len <= rx_cnt_ip_len;
		end
	end else begin
		rx_cnt_ip_len <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		ip_total_len <= 16'h0;
	end else if ( rx_state == IP_LEN ) begin
		if ( !rx_cnt_ip_len && s_mac_tvalid ) begin
			ip_total_len <= { s_mac_tdata, ip_total_len[7:0] };
		end else if ( s_mac_tvalid ) begin
			ip_total_len <= { ip_total_len[15:8], s_mac_tdata };
		end else begin
			ip_total_len <= ip_total_len;
		end
	end else begin
		ip_total_len <= ip_total_len;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_id <= 1'b0;
	end else if ( rx_state == IP_ID ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_id <= ~rx_cnt_ip_id;
		end else begin
			rx_cnt_ip_id <= rx_cnt_ip_id;
		end
	end else begin
		rx_cnt_ip_id <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		ip_id <= 16'h0;
	end else if ( rx_state == IP_ID ) begin
		if ( !rx_cnt_ip_id && s_mac_tvalid ) begin
			ip_id <= { s_mac_tdata, ip_id[7:0] };
		end else if ( s_mac_tvalid ) begin
			ip_id <= { ip_id[15:8], s_mac_tdata };
		end else begin
			ip_id <= ip_id;
		end
	end else begin
		ip_id <= ip_id;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_split <= 1'b0;
	end else if ( rx_state == IP_SPLIT ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_split <= ~rx_cnt_ip_split;
		end else begin
			rx_cnt_ip_split <= rx_cnt_ip_split;
		end
	end else begin
		rx_cnt_ip_split <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_check <= 1'b0;
	end else if ( rx_state == IP_CHECK ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_check <= ~rx_cnt_ip_check;
		end else begin
			rx_cnt_ip_check <= rx_cnt_ip_check;
		end
	end else begin
		rx_cnt_ip_check <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_addr <= 3'd0;
	end else if ( rx_state == IP_ADDR ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_addr <= rx_cnt_ip_addr + 3'd1;
		end else begin
			rx_cnt_ip_addr <= rx_cnt_ip_addr;
		end
	end else begin
		rx_cnt_ip_addr <= 3'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_ip_fill <= 6'd0;
	end else if ( rx_state == IP_FILL ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_ip_fill <= rx_cnt_ip_fill + 6'd1;
		end else begin
			rx_cnt_ip_fill <= rx_cnt_ip_fill;
		end
	end else begin
		rx_cnt_ip_fill <= 6'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_icmp_type <= 2'd0;
	end else if ( rx_state == ICMP_TYPE ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_icmp_type <= rx_cnt_icmp_type + 2'd1;
		end else begin
			rx_cnt_icmp_type <= rx_cnt_icmp_type;
		end
	end else begin
		rx_cnt_icmp_type <= 2'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_icmp_check <= 1'b0;
	end else if ( rx_state == ICMP_CHECK ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_icmp_check <= ~rx_cnt_icmp_check;
		end else begin
			rx_cnt_icmp_check <= rx_cnt_icmp_check;
		end
	end else begin
		rx_cnt_icmp_check <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_checksum <= 16'h0;
	end else if ( rx_state == ICMP_CHECK ) begin
		if ( !rx_cnt_icmp_check && s_mac_tvalid ) begin
			icmp_checksum <= { s_mac_tdata, icmp_checksum[7:0] };
		end else if ( s_mac_tvalid ) begin
			icmp_checksum <= { icmp_checksum[15:8], s_mac_tdata };
		end else begin
			icmp_checksum <= icmp_checksum;
		end
	end else begin
		icmp_checksum <= icmp_checksum;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_icmp_id <= 1'b0;
	end else if ( rx_state == ICMP_ID ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_icmp_id <= ~rx_cnt_icmp_id;
		end else begin
			rx_cnt_icmp_id <= rx_cnt_icmp_id;
		end
	end else begin
		rx_cnt_icmp_id <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_id <= 16'h0;
	end else if ( rx_state == ICMP_ID ) begin
		if ( !rx_cnt_icmp_id && s_mac_tvalid ) begin
			icmp_id <= { s_mac_tdata, icmp_id[7:0] };
		end else if ( s_mac_tvalid ) begin
			icmp_id <= { icmp_id[15:8], s_mac_tdata };
		end else begin
			icmp_id <= icmp_id;
		end
	end else begin
		icmp_id <= icmp_id;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_icmp_seq <= 2'd0;
	end else if ( rx_state == ICMP_SEQ ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_icmp_seq <= rx_cnt_icmp_seq + 2'd1;
		end else begin
			rx_cnt_icmp_seq <= rx_cnt_icmp_seq;
		end
	end else begin
		rx_cnt_icmp_seq <= 2'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_seq <= 16'h0;
	end else if ( rx_state == ICMP_SEQ ) begin
		if ( rx_cnt_icmp_seq == 2'd0 && s_mac_tvalid ) begin
			icmp_seq <= { s_mac_tdata, icmp_seq[7:0] };
		end else if ( rx_cnt_icmp_seq == 2'd1 && s_mac_tvalid ) begin
			icmp_seq <= { icmp_seq[15:8], s_mac_tdata };
		end else begin
			icmp_seq <= icmp_seq;
		end
	end else begin
		icmp_seq <= icmp_seq;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_cnt_icmp_data <= 12'd0;
	end else if ( rx_state == ICMP_DATA ) begin
		if ( s_mac_tvalid ) begin
			rx_cnt_icmp_data <= rx_cnt_icmp_data + 12'd1;
		end else begin
			rx_cnt_icmp_data <= rx_cnt_icmp_data;
		end
	end else begin
		rx_cnt_icmp_data <= 12'd0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_payload_len <= 16'd0;
	end else if ( rx_state == ICMP_SEQ && rx_cnt_icmp_seq == 2'd1 && s_mac_tvalid ) begin
		icmp_payload_len <= ( ip_total_len >= ( {10'd0, ip_header_len} + 16'd8 ) ) ?
			( ip_total_len - {10'd0, ip_header_len} - 16'd8 ) : 16'd0;
	end else begin
		icmp_payload_len <= icmp_payload_len;
	end
end

// ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓ ARP request crc32 check ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
	reg		[7:0]							rx_crc_data;
	reg										rx_crc_en;
	reg										rx_crc_end;
	reg										rx_crc_start;
	wire									rx_crc32_valid;
	wire	[31:0]							rx_crc32_temp;
	reg		[31:0]							rx_crc32;

CRC32_D8									u1_rx_CRC32_D8 (
	.sys_clk								( sys_clk		),
	.sys_rst_n								( sys_rst_n		),
	.data									( rx_crc_data	),
	.crc_start								( rx_crc_start	),
	.crc_en									( rx_crc_en		),
	.crc_end								( rx_crc_end	),
	.crc32									( rx_crc32_temp	),
	.crc32_valid							( rx_crc32_valid)
);

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_crc_start <= 1'b0;
	end else if ( rx_state == MAC_DES && rx_cnt_mac_des == 3'd0 && s_mac_tvalid ) begin
		rx_crc_start <= 1'b1;
	end else begin
		rx_crc_start <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_crc_end <= 1'b0;
	end else if ( rx_state == ARP_FILL && rx_cnt_arp_fill == 5'd17 && s_mac_tvalid ) begin
		rx_crc_end <= 1'b1;
	end else if ( rx_state == ICMP_DATA && rx_cnt_icmp_data >= icmp_data_cnt_max - 12'd1 && s_mac_tvalid ) begin
		rx_crc_end <= 1'b1;
	end else begin
		rx_crc_end <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_crc_en <= 1'b0;
	end else if ( rx_state == IDLE || rx_state == RX_SFD || rx_state == CRC || rx_state == ICMP_CRC ) begin
		rx_crc_en <= 1'b0;
	end else begin
		rx_crc_en <= s_mac_tvalid;
	end
end

always @ ( posedge sys_clk ) begin
	rx_crc_data <= s_mac_tdata;
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		rx_crc32 <= 32'h0;
	end else if ( rx_crc32_valid ) begin
		rx_crc32 <= rx_crc32_temp;
	end else begin
		rx_crc32 <= rx_crc32;
	end
end
// ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑ ARP request crc32 check ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_req <= 1'b0;
	end else if ( rx_state == IDLE ) begin									// receive corresponding IP address, enable arp_req
		arp_req <= 1'b0;
	end else if ( rx_cnt_arp_des_ip == 3'd4 && ip_des == board_ip_addr ) begin
		arp_req <= 1'b1;
	end else begin
		arp_req <= arp_req;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_req_true <= 1'b0;
	end else if ( rx_cnt_crc >= 3'd4 && rx_crc32_read == rx_crc32 ) begin
		arp_req_true <= arp_req;
	end else begin
		arp_req_true <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_pc_mac <= 48'h0;
	end else if ( arp_req_true ) begin
		arp_pc_mac <= mac_src;
	end else begin
		arp_pc_mac <= arp_pc_mac;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_pc_ip <= 32'h0;
	end else if ( arp_req_true ) begin
		arp_pc_ip <= ip_src;
	end else begin
		arp_pc_ip <= arp_pc_ip;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		arp_resp <= 1'b0;
	end else begin
		arp_resp <= arp_req_true;
	end
end

assign		arp_pc_refresh		=	arp_resp;

// -------------------------------- ICMP echo reply decision ------------------------------------------
// icmp_arm: echo request passes all header checks (type 0x08/code 0x00, ICMP protocol, dst IP is ours)
// and the TX is free. Payload is buffered only when armed; reply is generated only if CRC32 passes.
	reg										icmp_arm;
	reg										icmp_req_true;
	reg		[47:0]							icmp_pc_mac;
	reg		[31:0]							icmp_pc_ip;
	reg										icmp_tx_pending;				// reply request waiting for checksum pipeline

	wire									icmp_fifo_wr		= ( rx_state == ICMP_DATA ) && s_mac_tvalid && icmp_arm && ( rx_cnt_icmp_data < icmp_payload_len[11:0] );
	wire									icmp_fifo_clear		= ( rx_state == ICMP_SEQ ) && s_mac_tvalid && icmp_arm && ( rx_cnt_icmp_seq == 2'd1 );

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_arm <= 1'b0;
	end else if ( rx_state == IDLE ) begin								// frame done, clear arm
		icmp_arm <= 1'b0;
	end else if ( rx_state == ICMP_TYPE && rx_cnt_icmp_type == 2'd1 && s_mac_tvalid
				&& ( { s_mac_tdata_r, s_mac_tdata } == 16'h0800 ) ) begin	// echo request found, only reply when TX is free
		icmp_arm <= !tx_active;
	end else if ( rx_state == ICMP_SEQ && rx_cnt_icmp_seq == 2'd1 && s_mac_tvalid
				&& ( ip_total_len < ( ip_header_len + 8 ) ) ) begin			// invalid IP length, no reply
		icmp_arm <= 1'b0;
	end else if ( rx_state == ICMP_DATA && icmp_fifo_wr && icmp_fifo_full ) begin	// payload lost, no reply
		icmp_arm <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_req_true <= 1'b0;
	end else if ( rx_cnt_crc >= 3'd4 && rx_crc32_read == rx_crc32 ) begin
		icmp_req_true <= icmp_arm;
	end else begin
		icmp_req_true <= 1'b0;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_pc_mac <= 48'h0;
	end else if ( icmp_req_true ) begin
		icmp_pc_mac <= mac_src;
	end else begin
		icmp_pc_mac <= icmp_pc_mac;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_pc_ip <= 32'h0;
	end else if ( icmp_req_true ) begin
		icmp_pc_ip <= ip_src;
	end else begin
		icmp_pc_ip <= icmp_pc_ip;
	end
end

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_tx_pending <= 1'b0;
	end else if ( icmp_req_true ) begin
		icmp_tx_pending <= 1'b1;
	end else if ( icmp_tx_start && tx_start ) begin					// 仅当 TX 真正启动后清除, 避免与 ARP 回复竞争丢包
		icmp_tx_pending <= 1'b0;
	end
end

	wire									icmp_tx_start		= icmp_tx_pending && ( icmp_cks_stage == 3'd4 );	// 电平: 保持到 TX 真正启动
	wire									tx_start			= ( arp_resp || icmp_tx_start ) && !tx_active;

// -------------------------------- transform arp / icmp response ------------------------------------------
// ARP reply frame: preamble(7B 0x55 + SFD 0xD5) + eth_head(14B) + arp_head(8B) +
//                  sender MAC(6B) + sender IP(4B) + target MAC(6B) + target IP(4B) + CRC32(4B)
// byte index (tx_cnt): 0~7 preamble/SFD, 8~13 dest MAC, 14~19 src MAC, 20~28 arp_head array,
//                      29 opcode lo (0x02), 30~35 sender MAC, 36~39 sender IP,
//                      40~45 target MAC, 46~49 target IP, 50~67 padding 0x00, 68~71 CRC32 (LSB first)
// ICMP echo reply frame: preamble + dest MAC(6B) + src MAC(6B) + ethertype 0x0800 + IP header(20B)
//                        + ICMP header(8B) + echoed payload(NB) + CRC32(4B)
// byte index (tx_cnt): 0~7 preamble/SFD, 8~13 dest MAC, 14~19 src MAC, 20~21 ethertype,
//                      22~41 IP header, 42~49 ICMP header, 50~49+N payload, 50+N~53+N CRC32
// only the variable fields are replaced from the received request, fixed bytes are looked up
// from the eth_head / arp_head arrays, CRC32 is computed on the fly while shifting out

	localparam		TX_DATA_START		= 7'd8;								// first byte covered by CRC

	reg									tx_active;
	reg		[10:0]						tx_cnt;
	reg									tx_is_icmp;						// 1: ICMP echo reply, 0: ARP reply
	reg		[47:0]						tx_des_mac;
	reg		[31:0]						tx_des_ip;
	reg		[31:0]						tx_crc32;
	reg		[10:0]						tx_icmp_len;					// echoed payload length
	reg		[15:0]						tx_ip_cksum;					// IP header checksum of the reply
	reg		[15:0]						tx_icmp_cksum;					// ICMP checksum of the reply
	reg		[15:0]						tx_ip_id;						// IP identification
	reg		[15:0]						tx_icmp_id;						// ICMP identifier
	reg		[15:0]						tx_icmp_seq;					// ICMP sequence

	wire								tx_handshake		= txen && m_axis_tx.tready;
	wire	[10:0]						tx_frame_max		= tx_is_icmp ? ( 11'd53 + tx_icmp_len ) : 11'd71;	// last byte of the frame (ARP padded to 60B min)
	wire	[10:0]						tx_data_end			= tx_is_icmp ? ( 11'd49 + tx_icmp_len ) : 11'd67;	// last byte covered by CRC (ARP padding included)
	wire								tx_crc_start;
	wire								tx_crc_en;
	wire								tx_crc_end;

	function [7:0] arp_reply_byte( input [10:0] cnt );						// byte selection of the ARP reply frame
		if ( cnt <= 7'd6 ) begin											// preamble 0x55 x7
			arp_reply_byte = 8'h55;
		end else if ( cnt == 7'd7 ) begin									// SFD
			arp_reply_byte = 8'hD5;
		end else if ( cnt <= 7'd13 ) begin									// dest MAC = arp_pc_mac
			arp_reply_byte = tx_des_mac[ (47 - 8*(cnt - 7'd8)) -: 8 ];
		end else if ( cnt <= 7'd19 ) begin									// src MAC = eth_head[0~5]
			arp_reply_byte = eth_head[ cnt - 7'd14 ];
		end else if ( cnt <= 7'd28 ) begin									// ethertype + ARP head = arp_head[0~8]
			arp_reply_byte = arp_head[ cnt - 7'd20 ];
		end else if ( cnt == 7'd29 ) begin									// ARP opcode = 0x0002 (reply)
			arp_reply_byte = 8'h02;
		end else if ( cnt <= 7'd35 ) begin									// sender MAC = board_mac_addr
			arp_reply_byte = board_mac_addr[ (47 - 8*(cnt - 7'd30)) -: 8 ];
		end else if ( cnt <= 7'd39 ) begin									// sender IP = board_ip_addr
			arp_reply_byte = board_ip_addr[ (31 - 8*(cnt - 7'd36)) -: 8 ];
		end else if ( cnt <= 7'd45 ) begin									// target MAC = arp_pc_mac
			arp_reply_byte = tx_des_mac[ (47 - 8*(cnt - 7'd40)) -: 8 ];
		end else if ( cnt <= 7'd49 ) begin									// target IP = arp_pc_ip
			arp_reply_byte = tx_des_ip[ (31 - 8*(cnt - 7'd46)) -: 8 ];
		end else if ( cnt <= 7'd67 ) begin									// padding 0x00 to 60B minimum frame
			arp_reply_byte = 8'h00;
		end else begin														// CRC32, LSB first (Ethernet FCS order)
			arp_reply_byte = tx_crc32[ (8*(cnt - 7'd68)) +: 8 ];
		end
	endfunction

	wire	[15:0]							tx_total_w			= 16'd28 + { 5'b0, icmp_payload_len[10:0] };	// IP total length of the reply
	reg		[15:0]							tx_ip_total;													// latched at reply start

// IP header checksum = ~sum of 10 header words (checksum field counted as 0), ones-complement.
// Sum = 0x4500 + total + id + 0x0000 + 0x8001 + src_hi + src_lo + dst_hi + dst_lo
// constants folded into icmp_cks_const, computed in a 4-stage pipeline right after the
// frame CRC check (icmp_req_true), the reply TX starts when it completes
	wire		[19:0]						icmp_cks_const		= 20'h0_4500 + 20'h0_8001 + { 4'b0, board_ip_addr[31:16] } + { 4'b0, board_ip_addr[15:0] };

	reg		[2:0]							icmp_cks_stage;		// 0 idle, 1~4 pipeline running
	reg		[19:0]							icmp_sum_p1;		// stage 1: const + total + id
	reg		[19:0]							icmp_sum;			// stage 2: + dst IP
	reg		[19:0]							icmp_sum_f1;		// stage 3: fold
	reg		[15:0]							icmp_sum_f2;		// stage 4: fold, final value

	function automatic [15:0] fold_sum20(input [19:0] sum);
		reg [16:0] folded;
		begin
			folded = {1'b0, sum[15:0]} + sum[19:16];
			fold_sum20 = folded[15:0] + folded[16];
		end
	endfunction

always @ ( posedge sys_clk or negedge sys_rst_n ) begin
	if ( !sys_rst_n ) begin
		icmp_cks_stage	<= 3'd0;
		icmp_sum_p1		<= 20'd0;
		icmp_sum		<= 20'd0;
		icmp_sum_f1		<= 20'd0;
		icmp_sum_f2		<= 16'd0;
	end else if ( icmp_req_true ) begin
		icmp_cks_stage	<= 3'd1;
		icmp_sum_p1		<= icmp_cks_const + {4'b0, tx_total_w} + {4'b0, ip_id};
	end else if ( icmp_cks_stage == 3'd1 ) begin
		icmp_cks_stage	<= 3'd2;
		icmp_sum		<= icmp_sum_p1 + {4'b0, icmp_pc_ip[31:16]} + {4'b0, icmp_pc_ip[15:0]};
	end else if ( icmp_cks_stage == 3'd2 ) begin
		icmp_cks_stage	<= 3'd3;
		icmp_sum_f1		<= {4'b0, icmp_sum[15:0]} + {16'b0, icmp_sum[19:16]};
	end else if ( icmp_cks_stage == 3'd3 ) begin
		icmp_cks_stage	<= 3'd4;
		icmp_sum_f2		<= fold_sum20(icmp_sum_f1);
	end else if ( icmp_cks_stage == 3'd4 ) begin
		icmp_cks_stage	<= tx_start ? 3'd0 : 3'd4;				// 校验和就绪, 等待 TX 空闲后启动
	end else begin
		icmp_cks_stage	<= 3'd0;
	end
end

	wire	[15:0]							tx_ip_cksum_w		= ~icmp_sum_f2;

// ICMP checksum of the reply = checksum of the request + 0x0800 (only the type byte changes 0x08 -> 0x00),
// done in ones-complement arithmetic (end-around carry), same trick as udp_lean.sv
	wire	[15:0]							tx_icmp_cksum_w		= ( icmp_checksum >= 16'hF800 ) ? ( icmp_checksum + 16'h0800 + 16'd1 ) : ( icmp_checksum + 16'h0800 );

	function [7:0] icmp_reply_byte( input [10:0] cnt );						// byte selection of the ICMP echo reply frame
		if ( cnt <= 7'd6 ) begin											// preamble 0x55 x7
			icmp_reply_byte = 8'h55;
		end else if ( cnt == 7'd7 ) begin									// SFD
			icmp_reply_byte = 8'hD5;
		end else if ( cnt <= 7'd13 ) begin									// dest MAC = tx_des_mac (PC MAC)
			icmp_reply_byte = tx_des_mac[ (47 - 8*(cnt - 7'd8)) -: 8 ];
		end else if ( cnt <= 7'd19 ) begin									// src MAC = eth_head[0~5]
			icmp_reply_byte = eth_head[ cnt - 7'd14 ];
		end else if ( cnt <= 7'd21 ) begin									// ethertype = 0x0800 (IPv4)
			icmp_reply_byte = ( cnt == 7'd20 ) ? 8'h08 : 8'h00;
		end else if ( cnt <= 7'd23 ) begin									// version/IHL + TOS = 0x4500
			icmp_reply_byte = ( cnt == 7'd22 ) ? 8'h45 : 8'h00;
		end else if ( cnt <= 7'd25 ) begin									// IP total length
			icmp_reply_byte = tx_ip_total[ (15 - 8*(cnt - 7'd24)) -: 8 ];
		end else if ( cnt <= 7'd27 ) begin									// IP identification = rx ip_id
			icmp_reply_byte = tx_ip_id[ (15 - 8*(cnt - 7'd26)) -: 8 ];
		end else if ( cnt <= 7'd29 ) begin									// flags + fragment offset = 0x0000
			icmp_reply_byte = 8'h00;
		end else if ( cnt == 7'd30 ) begin									// TTL = 128
			icmp_reply_byte = 8'h80;
		end else if ( cnt == 7'd31 ) begin									// protocol = 1 (ICMP)
			icmp_reply_byte = 8'h01;
		end else if ( cnt <= 7'd33 ) begin									// IP header checksum
			icmp_reply_byte = tx_ip_cksum[ (15 - 8*(cnt - 7'd32)) -: 8 ];
		end else if ( cnt <= 7'd37 ) begin									// source IP = board_ip_addr
			icmp_reply_byte = board_ip_addr[ (31 - 8*(cnt - 7'd34)) -: 8 ];
		end else if ( cnt <= 7'd41 ) begin									// dest IP = tx_des_ip (PC IP)
			icmp_reply_byte = tx_des_ip[ (31 - 8*(cnt - 7'd38)) -: 8 ];
		end else if ( cnt <= 7'd43 ) begin									// ICMP type/code = 0x0000 (echo reply)
			icmp_reply_byte = 8'h00;
		end else if ( cnt <= 7'd45 ) begin									// ICMP checksum
			icmp_reply_byte = tx_icmp_cksum[ (15 - 8*(cnt - 7'd44)) -: 8 ];
		end else if ( cnt <= 7'd47 ) begin									// ICMP identifier
			icmp_reply_byte = tx_icmp_id[ (15 - 8*(cnt - 7'd46)) -: 8 ];
		end else if ( cnt <= 7'd49 ) begin									// ICMP sequence
			icmp_reply_byte = tx_icmp_seq[ (15 - 8*(cnt - 7'd48)) -: 8 ];
		end else if ( cnt <= ( 11'd49 + tx_icmp_len ) ) begin				// echoed payload from FIFO
			icmp_reply_byte = icmp_fifo_q;
		end else begin														// CRC32, LSB first (Ethernet FCS order)
			icmp_reply_byte = tx_crc32[ (8*(cnt - 11'd50 - tx_icmp_len)) +: 8 ];
		end
	endfunction

	wire									icmp_fifo_rd		= tx_handshake && tx_is_icmp && ( tx_cnt >= 11'd50 ) && ( tx_cnt <= tx_data_end );
	wire	[7:0]							icmp_fifo_q;
	wire									icmp_fifo_empty;
	wire									icmp_fifo_full;

fifo #(
	.DATA_WIDTH								( 8			),
	.DEPTH									( 2048		)
) u_icmp_payload_fifo (
	.clock									( sys_clk			),
	.rstn									( sys_rst_n			),
	.clear									( icmp_fifo_clear	),
	.data									( s_mac_tdata		),
	.wrreq									( icmp_fifo_wr		),
	.rdreq									( icmp_fifo_rd		),
	.empty									( icmp_fifo_empty	),
	.full									( icmp_fifo_full	),
	.q										( icmp_fifo_q		)
);

	assign		txdata				= tx_is_icmp ? icmp_reply_byte( tx_cnt ) : arp_reply_byte( tx_cnt );
	assign		arp_working			= tx_active;

	assign		tx_crc_start		= tx_active && tx_handshake && ( tx_cnt == TX_DATA_START );
	assign		tx_crc_en			= tx_active && tx_handshake && ( tx_cnt >= TX_DATA_START ) && ( tx_cnt <= tx_data_end );
	assign		tx_crc_end			= tx_active && tx_handshake && ( tx_cnt == tx_data_end );

	always @ ( posedge sys_clk or negedge sys_rst_n ) begin					// single counter drives the whole TX
		if ( !sys_rst_n ) begin
			tx_active		<= 1'b0;
			tx_cnt			<= 11'd0;
			tx_is_icmp		<= 1'b0;
			tx_des_mac		<= 48'h0;
			tx_des_ip		<= 32'h0;
			tx_crc32		<= 32'h0;
			tx_ip_total		<= 16'h0;
			tx_icmp_len		<= 11'd0;
			tx_ip_cksum		<= 16'h0;
			tx_icmp_cksum	<= 16'h0;
			tx_ip_id		<= 16'h0;
			tx_icmp_id		<= 16'h0;
			tx_icmp_seq		<= 16'h0;
		end else if ( tx_start ) begin								// start a new ARP / ICMP reply
			tx_active		<= 1'b1;
			tx_cnt			<= 11'd0;
			tx_is_icmp		<= icmp_tx_start;
			tx_des_mac		<= icmp_tx_start ? icmp_pc_mac : arp_pc_mac;
			tx_des_ip		<= icmp_tx_start ? icmp_pc_ip  : arp_pc_ip;
			tx_ip_total		<= tx_total_w;
			tx_icmp_len		<= ( icmp_payload_len >= 16'd18 ) ? icmp_payload_len[10:0] : 11'd18;	// pad short payloads to 60B min frame
			tx_ip_cksum		<= tx_ip_cksum_w;
			tx_icmp_cksum	<= tx_icmp_cksum_w;
			tx_ip_id		<= ip_id;
			tx_icmp_id		<= icmp_id;
			tx_icmp_seq		<= icmp_seq;
		end else if ( tx_active && m_axis_tx.tready ) begin
			tx_crc32		<= ( tx_cnt == tx_data_end ) ? tx_crc32_temp : tx_crc32;	// latch final CRC32
			tx_cnt			<= ( tx_cnt == tx_frame_max ) ? 11'd0 : tx_cnt + 11'd1;
			tx_active		<= ( tx_cnt != tx_frame_max );
		end
	end

	wire									tx_crc32_valid;
	wire	[31:0]							tx_crc32_temp;

	CRC32_D8								u2_tx_CRC32_D8 (
		.sys_clk							( sys_clk			),
		.sys_rst_n							( sys_rst_n			),
		.data								( txdata			),
		.crc_start							( tx_crc_start		),
		.crc_en								( tx_crc_en			),
		.crc_end							( tx_crc_end		),
		.crc32								( tx_crc32_temp		),
		.crc32_valid						( tx_crc32_valid	)
	);

endmodule

