`include "axis.svh"

// mii_axis: MII PHY <-> AXI-Stream 透明转换
// 数据通路: ETH核 <-> AXIS <-> MII
// 帧边界约定(tlast 为电平信号):
//   tlast 高电平 = 帧开始/帧内, 上升沿 = 帧开始
//   tlast 低电平 = 帧结束,     下降沿 = 帧结束
// RX: m_phy_rx 忽略 tready, 字节有效时拉高 tvalid
//     rxdv 低电平 = 帧结束, 拉低当拍直接结束帧
// TX: s_phy_tx 由本模块产生 tready 反压
// 透传模式: 前导码/SFD/CRC 均由上层负责
// 位序: 每字节低半字节先发/先收 (LSB first, 同以太网线序)

module phy_mii_axis(
    input  logic        rstn,
    //mii tx
    input  logic        tx_clk,
    output logic[3:0]   txd,
    output logic        txen,
    //mii rx
    input logic         rx_clk,
    input logic[3:0]    rxd,
    input logic         rxdv,
    axis.master         m_phy_rx,
    axis.slave          s_phy_tx
);

    parameter TX_IFG_CLKS   = 8'd24;                    // 帧间间隔 96 bit time @25MHz(100M MII), 0 为不强制

//=============================================================
// RX: 4bit 半字节(2 个时钟拼 1 字节, 低半字节先到) -> 8bit 字节
//     MII 的 rxdv 仅在整个帧数据期间拉高, 不似 RMII 的 crs_dv
//     会提前拉低, 因此 rxdv 下降沿即帧结束, 无需采样窗口:
//       - 帧中段拉低: 丢弃未拼满的半字节, tlast 直接拉低
//       - 帧间空闲: 保持空闲
//=============================================================
    logic       rx_in_frame;
    logic [1:0] rx_nib_cnt;
    logic [7:0] rx_shift;

    function [7:0] rx_shift_in(input [7:0] sh,
                               input [1:0] cnt,
                               input [3:0] d);
        case (cnt)
            2'd0:    rx_shift_in = {d, sh[3:0]};
            default: rx_shift_in = {sh[7:4], d};
        endcase
    endfunction

    assign m_phy_rx.tlast  = rx_in_frame;
    assign m_phy_rx.tvalid = rx_in_frame && (rx_nib_cnt == 2'd1);
    assign m_phy_rx.tdata  = rx_shift;
    assign m_phy_rx.tuser  = 1'b0;

    always_ff @(posedge rx_clk or negedge rstn) begin
        if (!rstn) begin
            rx_in_frame <= 1'b0;
            rx_nib_cnt  <= 2'd0;
            rx_shift    <= 8'h0;
        end else if (!rxdv) begin
            // rxdv 拉低 = 帧结束, 丢弃未拼满的半字节
            rx_in_frame <= 1'b0;
            rx_nib_cnt  <= 2'd0;
        end else if (rx_in_frame) begin
            // rxdv 高电平, 正常拼接
            rx_shift   <= rx_shift_in(rx_shift, rx_nib_cnt, rxd);
            rx_nib_cnt <= (rx_nib_cnt == 2'd1) ? 2'd0 : rx_nib_cnt + 2'd1;
        end else begin
            // rxdv 拉高 = 帧开始, 首个半字节为低半字节
            rx_in_frame <= 1'b1;
            rx_nib_cnt  <= 2'd0;
            rx_shift    <= {4'b0, rxd};
        end
    end

//=============================================================
// TX: 8bit 字节 -> 4bit 半字节, 每字节 2 拍发出 (低半字节先发)
//     帧内数据必须连续, 若 tvalid 中途断流则本帧作废丢弃
// tready 同一拍读取 tdata
//=============================================================
    logic       tx_buf_valid;
    logic [1:0] tx_nib_cnt;
    logic [7:0] tx_shift;
    logic [7:0] tx_ifg_cnt;
    logic       tx_abort;
    logic       tx_end_pending;

    // tlast is frame-level. Remember a falling edge until the byte currently
    // being shifted has completely left MII, and block next-frame prefetch.
    wire tx_end_seen = tx_end_pending ||
                       (tx_buf_valid && !s_phy_tx.tlast);
    wire tx_can_accept = !tx_end_seen &&
                         ((tx_nib_cnt == 2'd1) || !tx_buf_valid);
    wire tx_accept = s_phy_tx.tvalid && s_phy_tx.tlast &&
                     (tx_ifg_cnt == 8'd0) && tx_can_accept;

    assign s_phy_tx.tready = tx_abort || (tx_can_accept && (tx_ifg_cnt == 8'd0));
    assign txd = tx_shift[3:0];
    assign txen = tx_buf_valid;

    always_ff @(posedge tx_clk or negedge rstn) begin
        if (!rstn) begin
            tx_buf_valid <= 1'b0;
            tx_nib_cnt   <= 2'd0;
            tx_shift     <= 8'h0;
            tx_ifg_cnt   <= 8'd0;
            tx_abort     <= 1'b0;
            tx_end_pending <= 1'b0;
        end else begin
            if (tx_ifg_cnt != 8'd0)
                tx_ifg_cnt <= tx_ifg_cnt - 8'd1;

            if (tx_abort) begin
                // 丢弃本帧剩余数据, 直到 tlast 下降
                tx_end_pending <= 1'b0;
                if (!s_phy_tx.tlast) begin
                    tx_abort   <= 1'b0;
                    tx_ifg_cnt <= TX_IFG_CLKS;
                end
            end else if (tx_accept) begin
                tx_shift     <= s_phy_tx.tdata;
                tx_nib_cnt   <= 2'd0;
                tx_buf_valid <= 1'b1;
                tx_end_pending <= 1'b0;
            end else if (tx_buf_valid) begin
                if (tx_nib_cnt == 2'd1) begin
                    tx_buf_valid <= 1'b0;
                    tx_nib_cnt   <= 2'd0;
                    tx_shift     <= 8'h0;
                    tx_end_pending <= 1'b0;
                    if (tx_end_seen) begin
                        tx_ifg_cnt <= TX_IFG_CLKS;
                    end else begin
                        // tvalid 断流, 帧已无法恢复, 中止发送
                        tx_abort <= 1'b1;
                    end
                end else begin
                    tx_shift   <= {4'b0, tx_shift[7:4]};
                    tx_nib_cnt <= tx_nib_cnt + 2'd1;
                    if (!s_phy_tx.tlast)
                        tx_end_pending <= 1'b1;
                end
            end else begin
                tx_end_pending <= 1'b0;
            end
        end
    end
endmodule
