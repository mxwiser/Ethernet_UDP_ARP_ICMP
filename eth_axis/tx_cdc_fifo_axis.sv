`include "axis.svh"

// tx_cdc_fifo_axis: TX 数据通路跨时钟域 (clk -> tx_clk)
// sys_tx / udp_tx 在 clk 域 (来自 udp_axis_rx 的 ARP 应答 / udp_axis_tx 的 UDP 数据)
// phy_tx 在 tx_clk 域 (送往 rmii_axis), tlast 保持帧电平语义
// 每通道 9bit 异步 FIFO (dcfifo, 写 clk 域, 读 tx_clk 域):
//   [8]=1 为帧尾标记字, tlast 下降沿后写入一个 {1'b1, 8'h00}, 读侧弹出标记字即本帧结束
// 帧级仲裁 (sys 优先) 与读出在 tx_clk 域完成

module tx_cdc_fifo_axis(
    input logic      clk,
    input logic      tx_clk,
    input logic      rstn,
    axis.slave sys_tx,
    axis.slave udp_tx,
    axis.master phy_tx
);

// ================================ sys_tx 帧缓存 (clk 域) ================================
logic        sys_wr_full;
logic        sys_marker_pending;             // 帧尾标记字待写入
logic        sys_data_idx;                   // 上一拍 tlast (帧电平)
logic [8:0]  sys_fifo_data; 
assign sys_fifo_data = {1'b0,sys_tx.tdata};

assign sys_tx.tready = !sys_wr_full && !sys_marker_pending;

always_ff@(posedge clk or negedge rstn) begin
	if(!rstn) begin
		sys_data_idx       <= 'b0;
		sys_marker_pending <= 'b0;
	end else begin
		sys_data_idx <= sys_tx.tlast;
		if(!sys_tx.tlast && sys_data_idx) begin          // tlast 下降沿 = 帧结束
			sys_marker_pending <= 1'b1;
		end else if (sys_marker_pending && !sys_wr_full) begin
			sys_marker_pending <= 1'b0;                  // 标记字已写入 FIFO
		end
	end
end

wire  		 sys_empty;
wire  	 	 sys_rdreq;
wire[8:0]    sys_q;

user_dc_fifo_9b_1024d u_sys_tx_fifo(
	.wrclk		(clk),
	.aclr		(!rstn),
	.wrreq		(sys_marker_pending || sys_tx.tvalid),
	.data		(sys_marker_pending ? {1'b1,8'h00} : sys_fifo_data),
	.wrfull		(sys_wr_full),
	.rdclk		(tx_clk),
	.rdreq		(sys_rdreq),
	.q			(sys_q),
	.rdempty	(sys_empty)
);

// ================================ udp_tx 帧缓存 (clk 域) ================================
logic        udp_wr_full;
logic        udp_marker_pending;
logic        udp_data_idx;
logic [8:0]  udp_fifo_data; 
assign udp_fifo_data = {1'b0,udp_tx.tdata};

assign udp_tx.tready = !udp_wr_full && !udp_marker_pending;

always_ff@(posedge clk or negedge rstn) begin
	if(!rstn) begin
		udp_data_idx       <= 'b0;
		udp_marker_pending <= 'b0;
	end else begin
		udp_data_idx <= udp_tx.tlast;
		if(!udp_tx.tlast && udp_data_idx) begin          // tlast 下降沿 = 帧结束
			udp_marker_pending <= 1'b1;
		end else if (udp_marker_pending && !udp_wr_full) begin
			udp_marker_pending <= 1'b0;                  // 标记字已写入 FIFO
		end
	end
end

wire  		 udp_empty;
wire  	 	 udp_rdreq;
wire[8:0]    udp_q;

user_dc_fifo_9b_1024d u_udp_tx_fifo(
	.wrclk		(clk),
	.aclr		(!rstn),
	.wrreq		(udp_marker_pending || udp_tx.tvalid),
	.data		(udp_marker_pending ? {1'b1,8'h00} : udp_fifo_data),
	.wrfull		(udp_wr_full),
	.rdclk		(tx_clk),
	.rdreq		(udp_rdreq),
	.q			(udp_q),
	.rdempty	(udp_empty)
);

// ================================ 帧级仲裁 + 读出 (tx_clk 域) ================================
// dcfifo 读输出为寄存器: rdreq 弹出后下一拍 q 才有效, 用 rd_pending 等待一拍再捕获
// in_frame 标记本帧未结束: 帧尾标记字弹出前禁止切换通道, 避免帧间粘包
reg			 sel;							// 1'b0: sys, 1'b1: udp
logic		 rd_pending;					// 已弹出, 下一拍 cur_q 有效
logic		 in_frame;						// 帧进行中 (首个数据字到帧尾标记字)
logic		 out_valid;
logic [7:0]  out_data;
logic		 out_last;
wire		 cur_empty;
wire[8:0]    cur_q;
wire		 cur_rdreq;

assign cur_empty  = sel ? udp_empty : sys_empty;
assign cur_q      = sel ? udp_q     : sys_q;
assign cur_rdreq  = !cur_empty && !rd_pending && (!out_valid || phy_tx.tready);
assign sys_rdreq  = !sel ? cur_rdreq : 1'b0;
assign udp_rdreq  = sel  ? cur_rdreq : 1'b0;

always_ff@(posedge tx_clk or negedge rstn) begin
	if(!rstn) begin
		sel        <= 1'b0;
		rd_pending <= 1'b0;
		in_frame   <= 1'b0;
		out_valid  <= 1'b0;
		out_data   <= 8'h0;
		out_last   <= 1'b0;
	end else begin
		rd_pending <= cur_rdreq;
		if (rd_pending) begin
			if (cur_q[8]) begin
				// 帧尾标记字: 本帧结束, 选择下一帧: sys 优先
				in_frame  <= 1'b0;
				out_valid <= 1'b0;
				out_last  <= 1'b0;
				sel       <= sys_empty ? 1'b1 : 1'b0;
			end else begin
				in_frame  <= 1'b1;
				out_valid <= 1'b1;
				out_data  <= cur_q[7:0];
				out_last  <= 1'b1;
			end
		end else if (out_valid && !phy_tx.tready) begin
			out_valid <= out_valid;
			out_last  <= out_last;
		end else begin
			out_valid <= 1'b0;
			// FIFO read latency creates bubbles between bytes. tlast is a
			// frame-level signal and must remain asserted across those bubbles;
			// only the explicit marker above is allowed to end the frame.
			out_last  <= in_frame;
		end
		// 空闲且本帧已结束后才允许切换通道: sys 优先
		if (!in_frame && !rd_pending && cur_empty && !sys_empty) begin
			sel <= 1'b0;
		end else if (!in_frame && !rd_pending && cur_empty && sys_empty && !udp_empty) begin
			sel <= 1'b1;
		end
	end
end

assign phy_tx.tvalid = out_valid;
assign phy_tx.tdata  = out_data;
assign phy_tx.tlast  = out_last;
assign phy_tx.tuser  = 1'b0;
assign phy_tx.tkeep  = 1'b1;
assign phy_tx.tstrb  = 1'b1;

endmodule
