`timescale 1ns/1ps

// ARP stub: this test isolates the UDP receive parser.
module eth_axis (
    input wire sys_clk,
    input wire sys_rst_n,
    input wire [47:0] board_mac_addr,
    input wire [31:0] board_ip_addr,
    axis.slave s_axis_rx,
    axis.master m_axis_tx,
    output wire arp_working,
    output wire [47:0] arp_pc_mac,
    output wire [31:0] arp_pc_ip,
    output wire arp_pc_refresh
);
    assign s_axis_rx.tready = 1'b1;
    assign m_axis_tx.tdata = 8'h00;
    assign m_axis_tx.tvalid = 1'b0;
    assign m_axis_tx.tlast = 1'b0;
    assign m_axis_tx.tuser = 1'b0;
    assign arp_working = 1'b0;
    assign arp_pc_mac = 48'd0;
    assign arp_pc_ip = 32'd0;
    assign arp_pc_refresh = 1'b0;
endmodule

module tb_udp_rx_l144;
    localparam [47:0] BOARD_MAC = 48'h50_12_22_33_44_55;
    localparam [31:0] BOARD_IP  = 32'hc0_a8_c8_34;

    reg clk = 1'b0;
    reg rstn = 1'b0;
    always #5 clk = ~clk;

    axis rx_axis();
    axis arp_tx_axis();
    pc_head rx_head();

    wire udp_rxstart;
    wire udp_rxend;
    wire udp_rxframe_done;
    wire udp_rxdv;
    wire [7:0] udp_rxdata;
    wire [15:0] udp_rxamount;
    wire [15:0] udp_rxnum;
    wire arp_working;
    wire [31:0] test_count;

    reg [7:0] frame [0:8191];
    reg [7:0] received [0:4095];
    integer frame_size;
    integer received_count;
    integer start_count;
    integer end_count;
    integer done_count;
    integer command_count = 0;
    integer failures = 0;
    wire command_valid;
    wire [63:0] command_data;

    udp_axis_rx dut (
        .sys_clk(clk), .sys_rst_n(rstn),
        .board_mac_addr(BOARD_MAC), .board_ip_addr(BOARD_IP),
        .s_axis_rx(rx_axis), .m_axis_tx(arp_tx_axis),
        .arp_working(arp_working), .test_count(test_count),
        .udp_rxstart(udp_rxstart), .udp_rxend(udp_rxend),
        .udp_rxframe_done(udp_rxframe_done), .udp_rxdv(udp_rxdv),
        .udp_rxdata(udp_rxdata), .udp_rxamount(udp_rxamount),
        .udp_rxnum(udp_rxnum), .rx_head(rx_head)
    );

    udp_command_parser command_parser (
        .clk(clk), .rstn(rstn), .clear(1'b0),
        .udp_rxstart(udp_rxstart), .udp_rxend(udp_rxend),
        .udp_rxframe_done(udp_rxframe_done), .udp_rxdv(udp_rxdv),
        .udp_rxdata(udp_rxdata), .udp_rxamount(udp_rxamount),
        .command_ready(1'b1), .command_valid(command_valid),
        .command_data(command_data)
    );

    always @(posedge clk) begin
        if (rstn) begin
            if (udp_rxstart) start_count = start_count + 1;
            if (udp_rxend) end_count = end_count + 1;
            if (udp_rxframe_done) done_count = done_count + 1;
            if (command_valid) command_count = command_count + 1;
            if (udp_rxdv) begin
                received[received_count] = udp_rxdata;
                received_count = received_count + 1;
            end
        end
    end

    function automatic [31:0] crc32_byte(input [31:0] crc_in, input [7:0] datum);
        integer bit_index;
        reg [31:0] crc;
        begin
            crc = crc_in ^ datum;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
            crc32_byte = crc;
        end
    endfunction

    task automatic put_byte(input [7:0] datum, inout integer idx);
        begin
            frame[idx] = datum;
            idx = idx + 1;
        end
    endtask

    task automatic build_command_frame(input integer bad_fcs);
        integer i;
        integer payload_start;
        reg [31:0] command_crc;
        reg [31:0] ethernet_crc;
        begin
            build_frame(13, 5, 0, 8'h00);
            payload_start = 50;
            frame[payload_start+0] = 8'hff;
            frame[payload_start+1] = 8'h01;
            frame[payload_start+2] = 8'd1;
            frame[payload_start+3] = 8'd3;
            frame[payload_start+4] = 8'd4;
            frame[payload_start+5] = 8'h00;
            frame[payload_start+6] = 8'h00;
            frame[payload_start+7] = 8'h00;
            frame[payload_start+8] = 8'h64;
            command_crc = 32'hffffffff;
            for (i = 0; i < 9; i = i + 1)
                command_crc = crc32_byte(command_crc, frame[payload_start+i]);
            command_crc = command_crc ^ 32'hffffffff;
            frame[payload_start+9]  = command_crc[31:24];
            frame[payload_start+10] = command_crc[23:16];
            frame[payload_start+11] = command_crc[15:8];
            frame[payload_start+12] = command_crc[7:0];

            ethernet_crc = 32'hffffffff;
            for (i = 8; i < frame_size-4; i = i + 1)
                ethernet_crc = crc32_byte(ethernet_crc, frame[i]);
            ethernet_crc = ethernet_crc ^ 32'hffffffff;
            if (bad_fcs)
                ethernet_crc = ethernet_crc ^ 1;
            frame[frame_size-4] = ethernet_crc[7:0];
            frame[frame_size-3] = ethernet_crc[15:8];
            frame[frame_size-2] = ethernet_crc[23:16];
            frame[frame_size-1] = ethernet_crc[31:24];
        end
    endtask

    // malformed_kind: 0 valid, 1 bad FCS, 2 bad IP checksum,
    // 3 UDP length mismatch, 4 IPv4 fragment (MF), 5 invalid IHL.
    task automatic build_frame(
        input integer payload_len,
        input integer ihl_words,
        input integer malformed_kind,
        input [7:0] seed
    );
        integer i;
        integer idx;
        integer ip_start;
        integer ip_header_len;
        integer ip_total;
        integer eth_payload;
        integer udp_wire_len;
        integer checksum_sum;
        reg [15:0] checksum;
        reg [31:0] crc;
        begin
            idx = 0;
            for (i = 0; i < 7; i = i + 1) put_byte(8'h55, idx);
            put_byte(8'hd5, idx);
            put_byte(BOARD_MAC[47:40], idx);
            put_byte(BOARD_MAC[39:32], idx);
            put_byte(BOARD_MAC[31:24], idx);
            put_byte(BOARD_MAC[23:16], idx);
            put_byte(BOARD_MAC[15:8], idx);
            put_byte(BOARD_MAC[7:0], idx);
            put_byte(8'h02, idx); put_byte(8'h11, idx); put_byte(8'h22, idx);
            put_byte(8'h33, idx); put_byte(8'h44, idx); put_byte(8'h55, idx);
            put_byte(8'h08, idx); put_byte(8'h00, idx);

            ip_start = idx;
            ip_header_len = ihl_words * 4;
            ip_total = ip_header_len + 8 + payload_len;
            udp_wire_len = payload_len + 8;
            if (malformed_kind == 3)
                udp_wire_len = udp_wire_len + 1;

            put_byte({4'h4, ihl_words[3:0]}, idx);
            put_byte(8'h00, idx);
            put_byte(ip_total[15:8], idx); put_byte(ip_total[7:0], idx);
            put_byte(8'h12, idx); put_byte(8'h34, idx);
            if (malformed_kind == 4) begin
                put_byte(8'h20, idx); put_byte(8'h00, idx);
            end else begin
                put_byte(8'h00, idx); put_byte(8'h00, idx);
            end
            put_byte(8'h40, idx); put_byte(8'h11, idx);
            put_byte(8'h00, idx); put_byte(8'h00, idx);
            put_byte(8'hc0, idx); put_byte(8'ha8, idx);
            put_byte(8'hc8, idx); put_byte(8'h7b, idx);
            put_byte(BOARD_IP[31:24], idx); put_byte(BOARD_IP[23:16], idx);
            put_byte(BOARD_IP[15:8], idx); put_byte(BOARD_IP[7:0], idx);
            for (i = 20; i < ip_header_len; i = i + 1)
                put_byte(8'h80 + i, idx);

            checksum_sum = 0;
            for (i = 0; i < ip_header_len; i = i + 2)
                checksum_sum = checksum_sum + {frame[ip_start+i], frame[ip_start+i+1]};
            while (checksum_sum >> 16)
                checksum_sum = (checksum_sum & 16'hffff) + (checksum_sum >> 16);
            checksum = ~checksum_sum;
            frame[ip_start+10] = checksum[15:8];
            frame[ip_start+11] = checksum[7:0];
            if (malformed_kind == 2)
                frame[ip_start+10] = frame[ip_start+10] ^ 8'h01;

            put_byte(8'h27, idx); put_byte(8'h74, idx);
            put_byte(8'h27, idx); put_byte(8'h74, idx);
            put_byte(udp_wire_len[15:8], idx); put_byte(udp_wire_len[7:0], idx);
            put_byte(8'h00, idx); put_byte(8'h00, idx);
            for (i = 0; i < payload_len; i = i + 1)
                put_byte(seed + i, idx);

            eth_payload = ip_total;
            while (eth_payload < 46) begin
                put_byte(8'h00, idx);
                eth_payload = eth_payload + 1;
            end

            crc = 32'hffffffff;
            for (i = 8; i < idx; i = i + 1)
                crc = crc32_byte(crc, frame[i]);
            crc = crc ^ 32'hffffffff;
            if (malformed_kind == 1)
                crc = crc ^ 32'h00000001;
            put_byte(crc[7:0], idx); put_byte(crc[15:8], idx);
            put_byte(crc[23:16], idx); put_byte(crc[31:24], idx);
            frame_size = idx;
        end
    endtask

    task automatic clear_observation;
        begin
            received_count = 0;
            start_count = 0;
            end_count = 0;
            done_count = 0;
        end
    endtask

    task automatic transmit_frame;
        integer i;
        begin
            for (i = 0; i < frame_size; i = i + 1) begin
                @(negedge clk);
                rx_axis.tvalid = 1'b1;
                rx_axis.tlast = 1'b1;
                rx_axis.tdata = frame[i];
            end
            @(negedge clk);
            rx_axis.tvalid = 1'b0;
            rx_axis.tlast = 1'b0;
            repeat (8) @(posedge clk);
        end
    endtask

    task automatic check_good(input integer length, input integer ihl_words, input [7:0] seed);
        integer i;
        begin
            clear_observation();
            build_frame(length, ihl_words, 0, seed);
            transmit_frame();
            if (start_count != 1 || done_count != 1 || received_count != length ||
                end_count != ((length == 0) ? 0 : 1) || udp_rxamount != length) begin
                $display("RX_GOOD_FAIL len=%0d ihl=%0d start=%0d end=%0d done=%0d bytes=%0d amount=%0d",
                         length, ihl_words, start_count, end_count, done_count,
                         received_count, udp_rxamount);
                failures = failures + 1;
            end
            for (i = 0; i < length; i = i + 1) begin
                if (received[i] !== ((seed + i) & 8'hff)) begin
                    $display("RX_DATA_FAIL len=%0d index=%0d got=%02x expected=%02x",
                             length, i, received[i], ((seed + i) & 8'hff));
                    failures = failures + 1;
                end
            end
        end
    endtask

    task automatic check_rejected(input integer malformed_kind, input [127:0] name);
        begin
            clear_observation();
            build_frame(12, (malformed_kind == 5) ? 4 : 5, malformed_kind, 8'ha0);
            transmit_frame();
            if (malformed_kind == 1) begin
                // FCS is known only after payload streaming; the transactional
                // ring must discard it because frame_done remains low.
                if (start_count != 1 || received_count != 12 || done_count != 0) begin
                    $display("RX_REJECT_FAIL %0s start=%0d bytes=%0d done=%0d",
                             name, start_count, received_count, done_count);
                    failures = failures + 1;
                end
            end else if (start_count != 0 || received_count != 0 || done_count != 0) begin
                $display("RX_REJECT_FAIL %0s start=%0d bytes=%0d done=%0d",
                         name, start_count, received_count, done_count);
                failures = failures + 1;
            end
        end
    endtask

    integer length;
    initial begin
        rx_axis.tdata = 8'h00;
        rx_axis.tvalid = 1'b0;
        rx_axis.tlast = 1'b0;
        rx_axis.tuser = 1'b0;
        arp_tx_axis.tready = 1'b1;
        clear_observation();
        repeat (5) @(posedge clk);
        rstn = 1'b1;
        repeat (3) @(posedge clk);

        // Exhaust every legal non-fragmented payload length at a 1500-byte MTU.
        for (length = 0; length <= 1472; length = length + 1)
            check_good(length, 5, length[7:0]);
        check_good(12, 6, 8'h70); // IPv4 options

        check_rejected(1, "bad_fcs");
        check_rejected(2, "bad_ip_checksum");
        check_rejected(3, "udp_length_mismatch");
        check_rejected(4, "ipv4_fragment");
        check_rejected(5, "invalid_ihl");

        // RX start is intentionally one cycle ahead of the first payload byte
        // so the transactional ring can stage it. Verify the command parser's
        // start/data handshake and its final-FCS commit gate together.
        command_count = 0;
        clear_observation();
        build_command_frame(0);
        transmit_frame();
        repeat (3) @(posedge clk);
        if (command_count != 1 || command_data[63:62] != 2'd1 ||
            command_data[61:56] != 6'd3 || command_data[55:50] != 6'd4) begin
            $display("RX_COMMAND_COMMIT_FAIL count=%0d data=%016x", command_count, command_data);
            failures = failures + 1;
        end
        build_command_frame(1);
        transmit_frame();
        repeat (3) @(posedge clk);
        if (command_count != 1) begin
            $display("RX_BAD_FCS_COMMAND_FAIL count=%0d", command_count);
            failures = failures + 1;
        end

        if (failures == 0)
            $display("RX_REGRESSION_PASS good_lengths=1474 reject_cases=5 command_fcs_gate=pass");
        else
            $fatal(1, "RX_REGRESSION_FAIL failures=%0d", failures);
        $finish;
    end
endmodule
