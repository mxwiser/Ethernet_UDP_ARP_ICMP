`timescale 1ns/1ps

module tb_udp_tx_l144;
    localparam [47:0] BOARD_MAC = 48'h50_12_22_33_44_55;
    localparam [31:0] BOARD_IP  = 32'hc0_a8_c8_34;
    localparam [47:0] PC_MAC    = 48'h02_11_22_33_44_55;
    localparam [31:0] PC_IP     = 32'hc0_a8_c8_7b;

    reg clk = 0;
    reg rstn = 0;
    always #5 clk = ~clk;

    axis tx_axis();
    pc_head tx_head();
    reg txstart = 0;
    reg [15:0] txamount = 0;
    reg [7:0] payload [0:8191];
    integer req_index = 0;
    wire [7:0] txdata = payload[req_index];
    wire txreq;
    wire txbusy;

    reg [7:0] captured [0:7][0:2047];
    integer frame_length [0:7];
    integer frame_count = 0;
    integer current_length = 0;
    integer req_count = 0;
    integer failures = 0;
    reg frame_active = 0;
    reg random_ready = 0;
    reg [15:0] lfsr = 16'h1ace;

    udp_axis_tx dut (
        .sys_clk(clk), .sys_rst_n(rstn),
        .board_mac_addr(BOARD_MAC), .board_ip_addr(BOARD_IP),
        .m_axis_tx(tx_axis), .udp_txstart(txstart),
        .udp_txamount(txamount), .udp_txdata(txdata),
        .udp_txreq(txreq), .udp_txbusy(txbusy), .tx_head(tx_head)
    );

    always @(negedge clk) begin
        if (!rstn) begin
            tx_axis.tready <= 1'b1;
            lfsr <= 16'h1ace;
        end else if (random_ready) begin
            lfsr <= {lfsr[14:0], lfsr[15] ^ lfsr[13] ^ lfsr[12] ^ lfsr[10]};
            tx_axis.tready <= lfsr[0] | lfsr[2];
        end else begin
            tx_axis.tready <= 1'b1;
        end
    end

    always @(posedge clk) begin
        if (!rstn) begin
            req_index <= 0;
            req_count <= 0;
        end else if (txreq) begin
            req_index <= req_index + 1;
            req_count <= req_count + 1;
        end

        if (rstn && tx_axis.tvalid && tx_axis.tready) begin
            if (!frame_active) begin
                frame_active = 1;
                current_length = 0;
            end
            if (!tx_axis.tlast) begin
                $display("TX_TLAST_FAIL valid byte without frame level");
                failures = failures + 1;
            end
            captured[frame_count][current_length] = tx_axis.tdata;
            current_length = current_length + 1;
        end else if (rstn && frame_active && !tx_axis.tvalid) begin
            frame_length[frame_count] = current_length;
            frame_count = frame_count + 1;
            frame_active = 0;
            current_length = 0;
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

    task automatic clear_capture;
        integer i;
        begin
            frame_count = 0;
            current_length = 0;
            frame_active = 0;
            req_index = 0;
            req_count = 0;
            for (i = 0; i < 8; i = i + 1)
                frame_length[i] = 0;
        end
    endtask

    task automatic check_frame(input integer frame_number);
        integer i;
        integer ip_start;
        integer ip_total;
        integer wire_ip_bytes;
        integer expected_frame_length;
        integer checksum_sum;
        reg [31:0] crc;
        reg [31:0] received_fcs;
        begin
            ip_start = 22;
            ip_total = {captured[frame_number][ip_start+2], captured[frame_number][ip_start+3]};
            wire_ip_bytes = (ip_total < 46) ? 46 : ip_total;
            expected_frame_length = 8 + 14 + wire_ip_bytes + 4;
            if (frame_length[frame_number] != expected_frame_length) begin
                $display("TX_FRAME_LENGTH_FAIL frame=%0d got=%0d expected=%0d ip_total=%0d",
                         frame_number, frame_length[frame_number], expected_frame_length, ip_total);
                failures = failures + 1;
            end

            checksum_sum = 0;
            for (i = 0; i < 20; i = i + 2)
                checksum_sum = checksum_sum +
                    {captured[frame_number][ip_start+i], captured[frame_number][ip_start+i+1]};
            while (checksum_sum >> 16)
                checksum_sum = (checksum_sum & 16'hffff) + (checksum_sum >> 16);
            if ((checksum_sum & 16'hffff) != 16'hffff) begin
                $display("TX_IP_CHECKSUM_FAIL frame=%0d sum=%04x flags=%02x%02x",
                         frame_number, checksum_sum & 16'hffff,
                         captured[frame_number][ip_start+6], captured[frame_number][ip_start+7]);
                failures = failures + 1;
            end

            crc = 32'hffffffff;
            for (i = 8; i < frame_length[frame_number]-4; i = i + 1)
                crc = crc32_byte(crc, captured[frame_number][i]);
            crc = crc ^ 32'hffffffff;
            received_fcs = {
                captured[frame_number][frame_length[frame_number]-1],
                captured[frame_number][frame_length[frame_number]-2],
                captured[frame_number][frame_length[frame_number]-3],
                captured[frame_number][frame_length[frame_number]-4]
            };
            if (received_fcs !== crc) begin
                $display("TX_FCS_FAIL frame=%0d got=%08x expected=%08x",
                         frame_number, received_fcs, crc);
                failures = failures + 1;
            end

            // Padding must be deterministic zero, not bytes from the next FIFO packet.
            for (i = ip_start + ip_total; i < ip_start + wire_ip_bytes; i = i + 1) begin
                if (captured[frame_number][i] !== 8'h00) begin
                    $display("TX_PADDING_FAIL frame=%0d offset=%0d got=%02x",
                             frame_number, i, captured[frame_number][i]);
                    failures = failures + 1;
                end
            end
        end
    endtask

    task automatic verify_payload(input integer length, input [7:0] seed);
        integer f;
        integer i;
        integer ip_start;
        integer ip_total;
        integer flags_offset;
        integer fragment_data_start;
        integer fragment_data_len;
        integer payload_index;
        integer udp_length;
        begin
            payload_index = 0;
            for (f = 0; f < frame_count; f = f + 1) begin
                ip_start = 22;
                ip_total = {captured[f][ip_start+2], captured[f][ip_start+3]};
                flags_offset = {captured[f][ip_start+6], captured[f][ip_start+7]};
                if ((flags_offset & 13'h1fff) == 0) begin
                    udp_length = {captured[f][ip_start+24], captured[f][ip_start+25]};
                    if (udp_length != length + 8) begin
                        $display("TX_UDP_LENGTH_FAIL got=%0d expected=%0d", udp_length, length+8);
                        failures = failures + 1;
                    end
                    fragment_data_start = ip_start + 28;
                    fragment_data_len = ip_total - 28;
                end else begin
                    fragment_data_start = ip_start + 20;
                    fragment_data_len = ip_total - 20;
                end
                for (i = 0; i < fragment_data_len; i = i + 1) begin
                    if (payload_index >= length ||
                        captured[f][fragment_data_start+i] !== ((seed + payload_index) & 8'hff)) begin
                        $display("TX_PAYLOAD_FAIL frame=%0d index=%0d got=%02x expected=%02x",
                                 f, payload_index, captured[f][fragment_data_start+i],
                                 ((seed + payload_index) & 8'hff));
                        failures = failures + 1;
                    end
                    payload_index = payload_index + 1;
                end
            end
            if (payload_index != length) begin
                $display("TX_PAYLOAD_COUNT_FAIL got=%0d expected=%0d", payload_index, length);
                failures = failures + 1;
            end
        end
    endtask

    task automatic run_case(input integer length, input [7:0] seed, input integer use_backpressure);
        integer i;
        integer expected_frames;
        integer timeout;
        begin
            clear_capture();
            for (i = 0; i < length; i = i + 1)
                payload[i] = seed + i;
            // Make an accidental read beyond this packet visible in padding tests.
            payload[length] = 8'hde;
            txamount = length;
            random_ready = use_backpressure;

            @(negedge clk);
            txstart = 1;
            timeout = 0;
            while (!txbusy && timeout < 100) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            txstart = 0;
            timeout = 0;
            while (txbusy && timeout < 20000) begin
                @(negedge clk);
                timeout = timeout + 1;
            end
            random_ready = 0;
            tx_axis.tready = 1;
            repeat (4) @(posedge clk);
            if (timeout >= 20000) begin
                $display("TX_TIMEOUT len=%0d", length);
                failures = failures + 1;
            end

            expected_frames = (length + 8 + 1479) / 1480;
            if (expected_frames < 1) expected_frames = 1;
            if (frame_count != expected_frames) begin
                $display("TX_FRAME_COUNT_FAIL len=%0d got=%0d expected=%0d",
                         length, frame_count, expected_frames);
                failures = failures + 1;
            end
            if (req_count != length) begin
                $display("TX_REQ_COUNT_FAIL len=%0d req=%0d", length, req_count);
                failures = failures + 1;
            end
            for (i = 0; i < frame_count; i = i + 1)
                check_frame(i);
            verify_payload(length, seed);
            $display("TX_CASE len=%0d frames=%0d req=%0d backpressure=%0d",
                     length, frame_count, req_count, use_backpressure);
        end
    endtask

    initial begin
        tx_axis.tready = 1;
        tx_head.pc_mac_addr = PC_MAC;
        tx_head.pc_ip_addr = PC_IP;
        tx_head.pc_port = 16'd10100;
        tx_head.board_port = 16'd10100;
        repeat (5) @(posedge clk);
        rstn = 1;
        repeat (3) @(posedge clk);

        run_case(0, 8'h10, 0);
        run_case(1, 8'h20, 1);
        run_case(17, 8'h30, 1);
        run_case(18, 8'h40, 0);
        run_case(1472, 8'h50, 1);
        run_case(1473, 8'h60, 1);
        run_case(4000, 8'h70, 1);

        if (failures == 0)
            $display("TX_REGRESSION_PASS cases=7");
        else
            $fatal(1, "TX_REGRESSION_FAIL failures=%0d", failures);
        $finish;
    end
endmodule
