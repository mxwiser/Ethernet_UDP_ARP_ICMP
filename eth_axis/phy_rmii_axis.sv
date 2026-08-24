`include "axis.svh"

// rmii_axis: RMII PHY(LAN8720A) <-> AXI-Stream 透明转换
// 数据通路: ETH核 <-> AXIS <-> RMII
// 帧边界约定(tlast 为电平信号):
//   tlast 高电平 = 帧开始/帧内, 上升沿 = 帧开始
//   tlast 低电平 = 帧结束,     下降沿 = 帧结束
// RX: m_rmii_rx_axis_net 忽略 tready, 字节有效时拉高 tvalid
//     CRS_DV 会先于帧数据结束(尾部剩余数据组), 拉低后进入采样窗口
//     窗口内 tlast 保持高电平继续拼字节, 确认无剩余数据后才拉低 tlast
// TX: s_rmii_tx_axis_net 由本模块产生 tready 反压
// 透传模式: 前导码/SFD/CRC 均由上层负责

module phy_rmii_axis(
    input   wire            rstn,
    input   wire            rmii_clk,                   // 50 MHz
    // PHY RX
    input   wire            rmii_crs_dv,
    input   wire    [1:0]   rmii_rxdata,
    // PHY TX
    output  reg             rmii_txen,
    output  wire    [1:0]   rmii_txdata,
    output  wire            rmii_rst,                   // PHY 复位, 恒高
    // AXIS
    axis.master             m_rmii_rx_axis_net,         // tlast: 帧电平; 忽略 tready
    axis.slave              s_rmii_tx_axis_net          // tlast: 帧电平; 下降沿结束
);

    parameter RX_TAIL_CLKS  = 4'd2;                     // CRS_DV 拉低后继续采样确认帧结束的时钟数
    parameter TX_IFG_CLKS   = 8'd48;                    // 帧间间隔 96 bit time @ 50MHz, 0 为不强制

//=============================================================
// RX: 2bit 数据组(4 个时钟拼 1 字节) -> 8bit 字节, 跳过帧前空闲 0
//     帧尾处理: CRS_DV 是 CRS 与 RX_DV 的合并信号, 会先于帧数据
//     结束约 2 bit time, 尾部剩余的 2bit 数据组在 CRS_DV 拉低后
//     才到达, 且可能处于字节拼接的中间(半字节边界翻转)。因此
//     CRS_DV 拉低后进入采样窗口:
//       - tlast 保持高电平, tvalid 立即拉低
//       - 窗口内继续拼接数据组, 拼满的字节照常 tvalid 送出
//       - 窗口结束确认无剩余数据后, 才拉低 tlast
//=============================================================
    logic       rx_started;
    logic       rx_in_frame;
    logic       rx_tail;
    logic [3:0] rx_tail_cnt;
    logic [1:0] rx_nib_cnt;
    logic [7:0] rx_shift;

    function [7:0] rx_shift_in(input [7:0] sh,
                               input [1:0] cnt,
                               input [1:0] d);
        case (cnt)
            2'd0:    rx_shift_in = {sh[7:4], d, sh[1:0]};
            2'd1:    rx_shift_in = {sh[7:6], d, sh[3:0]};
            2'd2:    rx_shift_in = {d, sh[5:0]};
            default: rx_shift_in = {sh[7:2], d};
        endcase
    endfunction

    assign m_rmii_rx_axis_net.tlast  = rx_in_frame;
    assign m_rmii_rx_axis_net.tvalid = rx_in_frame && (rx_nib_cnt == 2'd3);
    assign m_rmii_rx_axis_net.tdata  = rx_shift;
    assign m_rmii_rx_axis_net.tuser  = 1'b0;
    assign m_rmii_rx_axis_net.tkeep  = 1'b1;
    assign m_rmii_rx_axis_net.tstrb  = 1'b1;

    always_ff @(posedge rmii_clk or negedge rstn) begin
        if (!rstn) begin
            rx_started  <= 1'b0;
            rx_in_frame <= 1'b0;
            rx_tail     <= 1'b0;
            rx_tail_cnt <= 4'd0;
            rx_nib_cnt  <= 2'd0;
            rx_shift    <= 8'h0;
        end else if (!rmii_crs_dv) begin
            if (rx_tail) begin
                // 采样窗口内, 继续拼接剩余数据
                if (rx_tail_cnt != 4'd0) begin
                    rx_tail_cnt <= rx_tail_cnt - 4'd1;
                    rx_shift    <= rx_shift_in(rx_shift, rx_nib_cnt, rmii_rxdata);
                    rx_nib_cnt  <= (rx_nib_cnt == 2'd3) ? 2'd0 : rx_nib_cnt + 2'd1;
                end else begin
                    // 窗口结束, 确认帧结束, 丢弃未拼满的半字节
                    rx_tail     <= 1'b0;
                    rx_in_frame <= 1'b0;
                    rx_started  <= 1'b0;
                    rx_nib_cnt  <= 2'd0;
                end
            end else if (rx_in_frame) begin
                // CRS_DV 先拉低: 进入采样窗口, 立即采样本拍剩余数据
                rx_tail     <= 1'b1;
                rx_tail_cnt <= RX_TAIL_CLKS - 4'd1;
                rx_shift    <= rx_shift_in(rx_shift, rx_nib_cnt, rmii_rxdata);
                rx_nib_cnt  <= (rx_nib_cnt == 2'd3) ? 2'd0 : rx_nib_cnt + 2'd1;
            end
            // 未成帧, 保持空闲
        end else if (rx_in_frame) begin
            // CRS_DV 高电平, 正常拼接
            rx_shift   <= rx_shift_in(rx_shift, rx_nib_cnt, rmii_rxdata);
            rx_nib_cnt <= (rx_nib_cnt == 2'd3) ? 2'd0 : rx_nib_cnt + 2'd1;
        end else if (!rx_started) begin
            if (rmii_rxdata != 2'b00) begin
                rx_started  <= 1'b1;
                rx_in_frame <= 1'b1;
                rx_nib_cnt  <= 2'd0;
                rx_shift    <= {6'b00, rmii_rxdata};
            end
        end
    end

//=============================================================
// TX: 8bit 字节 -> 2bit 数据组, 每字节 4 拍发出
//     帧内数据必须连续, 若 tvalid 中途断流则本帧作废丢弃
// tready 同一拍读取 tdata
//=============================================================
    logic       tx_buf_valid;
    logic [1:0] tx_nib_cnt;
    logic [7:0] tx_shift;
    logic [7:0] tx_ifg_cnt;
    logic       tx_abort;
    logic       tx_end_pending;

    // tlast is a frame-level signal in this project. The CDC can briefly
    // present its falling edge while the final byte is still being shifted,
    // then prefetch the first byte of the next frame. Latch that falling edge
    // until the current byte reaches its last dibit, and block the next-byte
    // handshake in the meantime.
    wire tx_end_seen = tx_end_pending || (tx_buf_valid && !s_rmii_tx_axis_net.tlast);
    wire tx_can_accept = !tx_end_seen &&
                         ((tx_nib_cnt == 2'd3) || !tx_buf_valid);
    wire tx_accept = s_rmii_tx_axis_net.tvalid && s_rmii_tx_axis_net.tlast &&
                     (tx_ifg_cnt == 8'd0) && tx_can_accept;

    assign s_rmii_tx_axis_net.tready = tx_abort || (tx_can_accept && (tx_ifg_cnt == 8'd0));
    assign rmii_txdata = tx_shift[1:0];
    assign rmii_rst    = 1'b1;

    always_ff @(posedge rmii_clk or negedge rstn) begin
        if (!rstn) begin
            tx_buf_valid <= 1'b0;
            tx_nib_cnt   <= 2'd0;
            tx_shift     <= 8'h0;
            rmii_txen    <= 1'b0;
            tx_ifg_cnt   <= 8'd0;
            tx_abort     <= 1'b0;
            tx_end_pending <= 1'b0;
        end else begin
            if (tx_ifg_cnt != 8'd0)
                tx_ifg_cnt <= tx_ifg_cnt - 8'd1;

            if (tx_abort) begin
                // 丢弃本帧剩余数据, 直到 tlast 下降
                tx_end_pending <= 1'b0;
                if (!s_rmii_tx_axis_net.tlast) begin
                    tx_abort   <= 1'b0;
                    tx_ifg_cnt <= TX_IFG_CLKS;
                end
            end else if (tx_accept) begin
                tx_shift     <= s_rmii_tx_axis_net.tdata;
                tx_nib_cnt   <= 2'd0;
                tx_buf_valid <= 1'b1;
                rmii_txen    <= 1'b1;
                tx_end_pending <= 1'b0;
            end else if (tx_buf_valid) begin
                if (tx_nib_cnt == 2'd3) begin
                    tx_buf_valid <= 1'b0;
                    tx_nib_cnt   <= 2'd0;
                    tx_shift     <= 8'h0;
                    tx_end_pending <= 1'b0;
                    if (tx_end_seen) begin
                        rmii_txen  <= 1'b0;
                        tx_ifg_cnt <= TX_IFG_CLKS;
                    end else begin
                        // tvalid 断流, 帧已无法恢复, 中止发送
                        rmii_txen <= 1'b0;
                        tx_abort  <= 1'b1;
                    end
                end else begin
                    tx_shift   <= {2'b00, tx_shift[7:2]};
                    tx_nib_cnt <= tx_nib_cnt + 2'd1;
                    if (!s_rmii_tx_axis_net.tlast)
                        tx_end_pending <= 1'b1;
                end
            end else begin
                tx_end_pending <= 1'b0;
            end
        end
    end
endmodule
