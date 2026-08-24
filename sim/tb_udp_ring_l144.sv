`timescale 1ns/1ps

module tb_udp_ring_l144;
    reg clk = 0;
    reg rstn = 0;
    always #5 clk = ~clk;

    reg rxstart = 0;
    reg frame_done = 0;
    reg rxdv = 0;
    reg [7:0] rxdata = 0;
    reg [15:0] rxamount = 0;
    reg txreq = 0;
    reg txbusy = 1;
    wire txstart;
    wire [15:0] txamount;
    wire [7:0] txdata;
    pc_head rx_head();
    pc_head tx_head();

    integer failures = 0;
    integer stress_index;
    integer stress_length;

    udp_ring #(.DATA_FIFO_DEPTH(8), .META_FIFO_DEPTH(4)) dut(
        .clk(clk), .rstn(rstn),
        .udp_rxstart(rxstart), .udp_rxframe_done(frame_done),
        .udp_rxdv(rxdv), .udp_rxdata(rxdata),
        .udp_rxamount(rxamount), .udp_rx_head(rx_head),
        .udp_txstart(txstart), .udp_txamount(txamount),
        .udp_txdata(txdata), .udp_txreq(txreq),
        .udp_txbusy(txbusy), .udp_tx_head(tx_head)
    );

    task automatic start_frame(input integer length, input [15:0] port_tag);
        begin
            @(negedge clk);
            rxamount = length;
            rx_head.pc_port = port_tag;
            rxstart = 1;
            @(negedge clk);
            rxstart = 0;
        end
    endtask

    task automatic send_bytes(input integer length, input [7:0] seed);
        integer i;
        begin
            for (i = 0; i < length; i = i + 1) begin
                rxdv = 1;
                rxdata = seed + i;
                @(negedge clk);
            end
            rxdv = 0;
        end
    endtask

    task automatic finish_frame;
        begin
            frame_done = 1;
            @(negedge clk);
            frame_done = 0;
            @(posedge clk);
        end
    endtask

    task automatic send_frame(
        input integer advertised_length,
        input integer actual_length,
        input [7:0] seed,
        input [15:0] port_tag
    );
        begin
            start_frame(advertised_length, port_tag);
            send_bytes(actual_length, seed);
            finish_frame();
        end
    endtask

    task automatic expect_head(input integer length, input [15:0] port_tag);
        begin
            #1;
            if (!txstart || txamount !== length[15:0] || tx_head.pc_port !== port_tag) begin
                $display("RING_HEAD_FAIL start=%0b amount=%0d/%0d port=%0d/%0d",
                         txstart, txamount, length, tx_head.pc_port, port_tag);
                failures = failures + 1;
            end
        end
    endtask

    task automatic pop_head;
        begin
            @(negedge clk);
            txbusy = 0;
            @(negedge clk);
            txbusy = 1;
        end
    endtask

    task automatic expect_byte(input [7:0] expected);
        begin
            #1;
            if (txdata !== expected) begin
                $display("RING_DATA_FAIL got=%02x expected=%02x", txdata, expected);
                failures = failures + 1;
            end
            @(negedge clk);
            txreq = 1;
            @(negedge clk);
            txreq = 0;
        end
    endtask

    task automatic drain_packet(input integer length, input [7:0] seed, input [15:0] port_tag);
        integer i;
        begin
            expect_head(length, port_tag);
            pop_head();
            for (i = 0; i < length; i = i + 1)
                expect_byte(seed + i);
        end
    endtask

    initial begin
        rx_head.pc_mac_addr = 48'h02_00_00_00_00_01;
        rx_head.pc_ip_addr = 32'hc0a8c87b;
        rx_head.pc_port = 16'd0;
        rx_head.board_port = 16'd10100;
        repeat (3) @(posedge clk);
        rstn = 1;
        repeat (2) @(posedge clk);

        // A packet that overruns the remaining space must not erase a packet
        // that was already committed.
        send_frame(4, 4, 8'ha0, 16'd1001);
        send_frame(5, 5, 8'hb0, 16'd1002); // 4 + 5 > depth 8: dropped
        drain_packet(4, 8'ha0, 16'd1001);
        #1;
        if (txstart) begin
            $display("RING_OVERFLOW_FAIL overflowing packet was committed");
            failures = failures + 1;
        end

        // A bad-FCS equivalent (start/data but no frame_done) is rolled back
        // when the next valid packet starts.
        start_frame(3, 16'd2001);
        send_bytes(3, 8'hc0);
        send_frame(3, 3, 8'hd0, 16'd2002);
        drain_packet(3, 8'hd0, 16'd2002);

        // Advertised/actual length mismatch must not become visible.
        send_frame(4, 3, 8'he0, 16'd3001);
        #1;
        if (txstart) begin
            $display("RING_LENGTH_FAIL mismatched packet was committed");
            failures = failures + 1;
        end

        // Zero-length packets carry metadata but consume no data byte.  The
        // following one-byte packet must therefore begin with its own byte.
        send_frame(0, 0, 8'h00, 16'd4001);
        send_frame(1, 1, 8'hf1, 16'd4002);
        drain_packet(0, 8'h00, 16'd4001);
        drain_packet(1, 8'hf1, 16'd4002);

        // Wrap both data and metadata pointers repeatedly.
        send_frame(2, 2, 8'h10, 16'd5001);
        send_frame(2, 2, 8'h20, 16'd5002);
        send_frame(2, 2, 8'h30, 16'd5003);
        drain_packet(2, 8'h10, 16'd5001);
        drain_packet(2, 8'h20, 16'd5002);
        drain_packet(2, 8'h30, 16'd5003);

        // Long pointer-wrap regression across every representable occupancy
        // size of this deliberately tiny depth-8 test instance.
        for (stress_index = 0; stress_index < 2000; stress_index = stress_index + 1) begin
            stress_length = (stress_index * 13) % 8;
            send_frame(stress_length, stress_length, stress_index[7:0],
                       16'd6000 + stress_index[15:0]);
            drain_packet(stress_length, stress_index[7:0],
                         16'd6000 + stress_index[15:0]);
        end

        #1;
        if (txstart) begin
            $display("RING_DRAIN_FAIL metadata remains after drain");
            failures = failures + 1;
        end

        if (failures == 0)
            $display("RING_TRANSACTION_REGRESSION_PASS stress_packets=2000");
        else
            $fatal(1, "RING_TRANSACTION_REGRESSION_FAIL failures=%0d", failures);
        $finish;
    end
endmodule
