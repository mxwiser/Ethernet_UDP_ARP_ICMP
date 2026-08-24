`timescale 1ns/1ps

module user_dc_fifo_9b_1024d (
    input wire aclr,
    input wire [8:0] data,
    input wire rdclk,
    input wire rdreq,
    input wire wrclk,
    input wire wrreq,
    output reg [8:0] q,
    output wire rdempty,
    output wire wrfull
);
    reg [8:0] mem [0:31];
    integer wrptr = 0;
    integer rdptr = 0;
    assign rdempty = (wrptr == rdptr);
    assign wrfull = 1'b0;
    always @(posedge wrclk or posedge aclr) begin
        if (aclr) wrptr <= 0;
        else if (wrreq) begin
            mem[wrptr & 31] <= data;
            wrptr <= wrptr + 1;
        end
    end
    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin rdptr <= 0; q <= 0; end
        else if (rdreq && !rdempty) begin
            q <= mem[rdptr & 31];
            rdptr <= rdptr + 1;
        end
    end
endmodule

module tb_rx_cdc_l144;
    reg rx_clk = 0;
    reg sys_clk = 0;
    reg rstn = 0;
    always #7 rx_clk = ~rx_clk;
    always #10 sys_clk = ~sys_clk;
    axis s_rx();
    axis m_rx();
    integer bytes = 0;

    rx_cdc_fifo_axis dut(
        .rstn(rstn), .clk(sys_clk), .rx_clk(rx_clk), .s_rx(s_rx), .m_rx(m_rx)
    );

    always @(posedge sys_clk)
        if (rstn && m_rx.tvalid && m_rx.tready) bytes = bytes + 1;

    initial begin
        m_rx.tready = 1'b1;
        s_rx.tvalid = 1'b0;
        s_rx.tlast = 1'b0;
        s_rx.tdata = 0;
        s_rx.tuser = 0;
        repeat (4) @(posedge sys_clk);
        rstn = 1'b1;
        @(negedge rx_clk);
        s_rx.tlast = 1'b1;
        s_rx.tvalid = 1'b1;
        s_rx.tdata = 8'h11;
        @(negedge rx_clk); s_rx.tdata = 8'h22;
        @(negedge rx_clk); s_rx.tdata = 8'h33;
        @(negedge rx_clk);
        s_rx.tvalid = 1'b0;
        s_rx.tlast = 1'b0;
        wait (bytes == 3);
        repeat (8) @(posedge sys_clk);
        $display("L144_RX_CDC bytes=%0d valid=%0b tlast=%0b", bytes, m_rx.tvalid, m_rx.tlast);
        if (m_rx.tlast)
            $display("RISK_CONFIRMED RX CDC lost the frame-end falling edge");
        else
            $display("L144_RX_CDC_BOUNDARY_PASS");
        $finish;
    end
endmodule
