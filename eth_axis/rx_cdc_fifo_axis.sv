`include "axis.svh"

// rx_cdc_fifo_axis: RX 数据通路跨时钟域 (rx_clk -> clk)
// s_rx 在 rx_clk 域(来自 rmii_axis), m_rx 在 clk 域(送往 udp_axis_rx / eth_axis)
// tlast 为帧电平信号(高电平 = 帧内), 逐字节与 tdata 一起存入异步 FIFO,
// 读侧原样还原 tlast 电平, 帧边界不失真
// dcfifo 为纯 RTL 异步 FIFO, 格雷码指针 + 双触发器同步

module rx_cdc_fifo_axis(
    input logic rstn,
    input logic clk,
    input logic rx_clk,
    axis.slave s_rx,
    axis.master m_rx
);
    parameter DEPTH = 512;

    logic        wr_full;
    logic        rd_empty;
    logic [8:0]  rd_q;
    logic        rd_req;
    logic        rd_pending;          // 已弹出, q 下一拍有效
    logic        out_valid;
    logic [7:0]  out_data;
    logic        out_last;
	logic        frame_level_d;
	logic        end_pending;
	wire         frame_end_seen = frame_level_d && !s_rx.tlast;
	wire         write_end = end_pending || frame_end_seen;
	wire         fifo_wrreq = !wr_full && (write_end || s_rx.tvalid);
	wire [8:0]   fifo_wrdata = write_end ? 9'h100 : {1'b0, s_rx.tdata};

// ================================ 写侧: rx_clk 域 ================================
	// End markers have priority over bytes, preserving tlast's falling edge.
    assign s_rx.tready = !wr_full && !write_end;

	always_ff @(posedge rx_clk or negedge rstn) begin
		if (!rstn) begin
			frame_level_d <= 1'b0;
			end_pending <= 1'b0;
		end else begin
			frame_level_d <= s_rx.tlast;
			if (write_end && wr_full)
				end_pending <= 1'b1;
			else if (write_end && !wr_full)
				end_pending <= 1'b0;
		end
	end

    user_dc_fifo_9b_1024d  u_rx_dcfifo (
        .wrclk   (rx_clk),
		.wrreq   (fifo_wrreq),
		.data    (fifo_wrdata),
        .wrfull  (wr_full),
        .rdclk   (clk),
        .rdreq   (rd_req),
        .q       (rd_q),
        .rdempty (rd_empty),
        .aclr    (!rstn)
    );

// ================================ 读侧: clk 域 ================================
// dcfifo 读输出为寄存器: rd_req 拉高弹出数据, 下一拍 rd_q 才有效,
// 用 rd_pending 标记该等待周期; 捕获后 out_valid 拉高与 tready 握手
// out_valid && tready 与下一字弹出同拍进行, 支持背靠背连续传输
    assign rd_req = !rd_empty && !rd_pending && (!out_valid || m_rx.tready);

    always_ff @(posedge clk or negedge rstn) begin
        if (!rstn) begin
            rd_pending <= 1'b0;
            out_valid  <= 1'b0;
            out_data   <= 8'h0;
            out_last   <= 1'b0;
        end else begin
            rd_pending <= rd_req;
            if (rd_pending) begin
				if (rd_q[8]) begin
					out_valid <= 1'b0;
					out_last  <= 1'b0;
				end else begin
					out_valid <= 1'b1;
					out_data  <= rd_q[7:0];
					out_last  <= 1'b1;
				end
            end else if (out_valid && !m_rx.tready) begin
                out_valid <= out_valid;
            end else begin
                out_valid <= 1'b0;
            end
        end
    end

    assign m_rx.tvalid = out_valid;
    assign m_rx.tdata  = out_data;
    assign m_rx.tlast  = out_last;
    assign m_rx.tuser  = 1'b0;

endmodule
