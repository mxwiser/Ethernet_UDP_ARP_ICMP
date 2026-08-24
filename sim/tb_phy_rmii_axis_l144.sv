`timescale 1ns/1ps

module tb_phy_rmii_axis_l144;
    reg clk = 0;
    reg rstn = 0;
    always #10 clk = ~clk; // 50 MHz

    reg crs_dv = 0;
    reg [1:0] rxdata = 0;
    wire txen;
    wire [1:0] txdata;
    wire phy_rst;
    axis rx_axis();
    axis tx_axis();

    reg [7:0] expected [0:255];
    reg [7:0] tx_seen [0:255];
    reg [7:0] rx_seen [0:255];
    integer tx_byte_count = 0;
    integer rx_byte_count = 0;
    integer tx_dibit_count = 0;
    integer failures = 0;

    phy_rmii_axis #(.RX_TAIL_CLKS(2), .TX_IFG_CLKS(48)) dut (
        .rstn(rstn), .rmii_clk(clk),
        .rmii_crs_dv(crs_dv), .rmii_rxdata(rxdata),
        .rmii_txen(txen), .rmii_txdata(txdata), .rmii_rst(phy_rst),
        .m_rmii_rx_axis_net(rx_axis), .s_rmii_tx_axis_net(tx_axis)
    );

    always @(posedge clk) begin
        if (rstn && txen) begin
            case (tx_dibit_count)
                0: tx_seen[tx_byte_count][1:0] = txdata;
                1: tx_seen[tx_byte_count][3:2] = txdata;
                2: tx_seen[tx_byte_count][5:4] = txdata;
                3: begin
                    tx_seen[tx_byte_count][7:6] = txdata;
                    tx_byte_count = tx_byte_count + 1;
                end
            endcase
            tx_dibit_count = (tx_dibit_count == 3) ? 0 : tx_dibit_count + 1;
        end
        if (rstn && rx_axis.tvalid) begin
            if (!rx_axis.tlast) begin
                $display("RMII_RX_LEVEL_FAIL valid without tlast");
                failures = failures + 1;
            end
            rx_seen[rx_byte_count] = rx_axis.tdata;
            rx_byte_count = rx_byte_count + 1;
        end
    end

    task automatic fill_expected(input integer length, input [7:0] seed);
        integer i;
        begin
            for (i = 0; i < length; i = i + 1)
                expected[i] = seed + i;
            expected[0] = 8'h55; // first dibit must be non-zero for RX start detect
        end
    endtask

    task automatic send_axis_frame(input integer length);
        integer i;
        begin
            i = 0;
            @(negedge clk);
            tx_axis.tvalid = 1;
            tx_axis.tlast = 1;
            tx_axis.tdata = expected[0];
            while (i < length) begin
                @(posedge clk);
                if (tx_axis.tready) begin
                    i = i + 1;
                    @(negedge clk);
                    if (i < length)
                        tx_axis.tdata = expected[i];
                    else begin
                        tx_axis.tvalid = 0;
                        tx_axis.tlast = 0;
                    end
                end
            end
            wait (!txen);
            repeat (3) @(posedge clk);
        end
    endtask

    task automatic send_rmii_frame(input integer length, input integer early_drop);
        integer i;
        integer dibit;
        begin
            for (i = 0; i < length; i = i + 1) begin
                for (dibit = 0; dibit < 4; dibit = dibit + 1) begin
                    @(negedge clk);
                    rxdata = (expected[i] >> (2*dibit)) & 2'b11;
                    crs_dv = !(early_drop && i == length-1 && dibit >= 2);
                end
            end
            @(negedge clk);
            crs_dv = 0;
            rxdata = 0;
            repeat (5) @(posedge clk);
        end
    endtask

    task automatic check_bytes(input integer is_tx, input integer length, input [127:0] label_text);
        integer i;
        integer count;
        begin
            count = is_tx ? tx_byte_count : rx_byte_count;
            if (count != length) begin
                $display("RMII_%0s_COUNT_FAIL got=%0d expected=%0d", label_text, count, length);
                failures = failures + 1;
            end
            for (i = 0; i < length; i = i + 1) begin
                if ((is_tx ? tx_seen[i] : rx_seen[i]) !== expected[i]) begin
                    $display("RMII_%0s_DATA_FAIL index=%0d got=%02x expected=%02x", label_text, i,
                             is_tx ? tx_seen[i] : rx_seen[i], expected[i]);
                    failures = failures + 1;
                end
            end
        end
    endtask

    initial begin
        tx_axis.tvalid = 0;
        tx_axis.tlast = 0;
        tx_axis.tdata = 0;
        tx_axis.tuser = 0;
        tx_axis.tkeep = 1;
        tx_axis.tstrb = 1;
        repeat (4) @(posedge clk);
        rstn = 1;
        repeat (3) @(posedge clk);

        fill_expected(72, 8'h20);
        tx_byte_count = 0;
        tx_dibit_count = 0;
        send_axis_frame(72);
        check_bytes(1, 72, "TX");

        fill_expected(72, 8'h40);
        rx_byte_count = 0;
        send_rmii_frame(72, 0);
        check_bytes(0, 72, "RX_NORMAL");
        if (rx_axis.tlast) begin
            $display("RMII_RX_END_FAIL tlast stuck high");
            failures = failures + 1;
        end

        // LAN8720-style CRS_DV deassertion can precede the final two dibits.
        fill_expected(61, 8'h80);
        rx_byte_count = 0;
        send_rmii_frame(61, 1);
        check_bytes(0, 61, "RX_EARLY_DROP");

        if (!phy_rst) begin
            $display("RMII_PHY_RESET_FAIL");
            failures = failures + 1;
        end
        if (failures == 0)
            $display("RMII_REGRESSION_PASS tx=72 rx_normal=72 rx_early_drop=61");
        else
            $fatal(1, "RMII_REGRESSION_FAIL failures=%0d", failures);
        $finish;
    end
endmodule
