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
    reg [8:0] mem [0:63];
    integer wrptr = 0;
    integer rdptr = 0;
    assign rdempty = (wrptr == rdptr);
    assign wrfull = ((wrptr - rdptr) >= 63);
    always @(posedge wrclk or posedge aclr) begin
        if (aclr) wrptr <= 0;
        else if (wrreq && !wrfull) begin
            mem[wrptr & 63] <= data;
            wrptr <= wrptr + 1;
        end
    end
    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin rdptr <= 0; q <= 0; end
        else if (rdreq && !rdempty) begin
            q <= mem[rdptr & 63];
            rdptr <= rdptr + 1;
        end
    end
endmodule

module tb_tx_cdc_l144;
    reg sys_clk = 0;
    reg tx_clk = 0;
    reg rstn = 0;
    always #5 sys_clk = ~sys_clk;
    always #7 tx_clk = ~tx_clk;

    axis sys_tx();
    axis udp_tx();
    axis phy_tx();
    reg [7:0] seen [0:3][0:127];
    integer seen_len [0:3];
    integer frame_count = 0;
    integer current_len = 0;
    integer failures = 0;
    reg in_frame = 0;
    reg [15:0] lfsr = 16'h6d3a;

    tx_cdc_fifo_axis dut(
        .clk(sys_clk), .tx_clk(tx_clk), .rstn(rstn),
        .sys_tx(sys_tx), .udp_tx(udp_tx), .phy_tx(phy_tx)
    );

    always @(negedge tx_clk) begin
        if (!rstn) begin
            phy_tx.tready <= 1;
            lfsr <= 16'h6d3a;
        end else begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            phy_tx.tready <= lfsr[0] | lfsr[3];
        end
    end

    always @(posedge tx_clk) begin
        if (rstn && phy_tx.tvalid && phy_tx.tready) begin
            if (!in_frame) begin
                in_frame = 1;
                current_len = 0;
            end
            if (!phy_tx.tlast) begin
                $display("TX_CDC_LEVEL_FAIL");
                failures = failures + 1;
            end
            seen[frame_count][current_len] = phy_tx.tdata;
            current_len = current_len + 1;
        end else if (rstn && in_frame && !phy_tx.tlast) begin
            seen_len[frame_count] = current_len;
            $display("TX_CDC_FRAME frame=%0d len=%0d", frame_count, current_len);
            frame_count = frame_count + 1;
            current_len = 0;
            in_frame = 0;
        end
    end

    initial begin
        repeat (100000) @(posedge sys_clk);
        $fatal(1, "TX_CDC_TIMEOUT frames=%0d sys_valid=%0b udp_valid=%0b phy_valid=%0b phy_last=%0b",
               frame_count, sys_tx.tvalid, udp_tx.tvalid, phy_tx.tvalid, phy_tx.tlast);
    end

    task automatic send_sys_frame(input integer length, input [7:0] seed);
        integer i;
        begin
            i = 0;
            @(negedge sys_clk);
            sys_tx.tvalid = 1;
            sys_tx.tlast = 1;
            sys_tx.tdata = seed;
            while (i < length) begin
                @(posedge sys_clk);
                if (sys_tx.tready) begin
                    i = i + 1;
                    @(negedge sys_clk);
                    if (i < length) sys_tx.tdata = seed + i;
                    else begin sys_tx.tvalid = 0; sys_tx.tlast = 0; end
                end
            end
        end
    endtask

    task automatic send_udp_frame(input integer length, input [7:0] seed);
        integer i;
        begin
            i = 0;
            @(negedge sys_clk);
            udp_tx.tvalid = 1;
            udp_tx.tlast = 1;
            udp_tx.tdata = seed;
            while (i < length) begin
                @(posedge sys_clk);
                if (udp_tx.tready) begin
                    i = i + 1;
                    @(negedge sys_clk);
                    if (i < length) udp_tx.tdata = seed + i;
                    else begin udp_tx.tvalid = 0; udp_tx.tlast = 0; end
                end
            end
        end
    endtask

    task automatic check_frame(input integer frame_no, input integer length, input [7:0] seed);
        integer i;
        begin
            if (seen_len[frame_no] != length) begin
                $display("TX_CDC_COUNT_FAIL frame=%0d got=%0d expected=%0d",
                         frame_no, seen_len[frame_no], length);
                failures = failures + 1;
            end
            for (i = 0; i < length; i = i + 1) begin
                if (seen[frame_no][i] !== ((seed + i) & 8'hff)) begin
                    $display("TX_CDC_DATA_FAIL frame=%0d index=%0d got=%02x expected=%02x",
                             frame_no, i, seen[frame_no][i], ((seed+i)&8'hff));
                    failures = failures + 1;
                end
            end
        end
    endtask

    initial begin
        sys_tx.tvalid = 0; sys_tx.tlast = 0; sys_tx.tdata = 0;
        sys_tx.tuser = 0; sys_tx.tkeep = 1; sys_tx.tstrb = 1;
        udp_tx.tvalid = 0; udp_tx.tlast = 0; udp_tx.tdata = 0;
        udp_tx.tuser = 0; udp_tx.tkeep = 1; udp_tx.tstrb = 1;
        phy_tx.tready = 1;
        repeat (5) @(posedge sys_clk);
        rstn = 1;
        repeat (4) @(posedge sys_clk);

        // Both queues become non-empty together: the ARP/ICMP channel has
        // frame-level priority, but UDP must follow without interleaving.
        fork
            send_sys_frame(23, 8'h10);
            send_udp_frame(31, 8'h80);
        join
        wait (frame_count == 2);

        // Back-to-back UDP frames exercise marker insertion while the next
        // producer is already waiting on tready.
        send_udp_frame(7, 8'hc0);
        send_udp_frame(9, 8'he0);
        wait (frame_count == 4);
        repeat (5) @(posedge tx_clk);

        check_frame(0, 23, 8'h10);
        check_frame(1, 31, 8'h80);
        check_frame(2, 7, 8'hc0);
        check_frame(3, 9, 8'he0);

        if (failures == 0)
            $display("TX_CDC_REGRESSION_PASS frames=4 random_backpressure=on");
        else
            $fatal(1, "TX_CDC_REGRESSION_FAIL failures=%0d", failures);
        $finish;
    end
endmodule
