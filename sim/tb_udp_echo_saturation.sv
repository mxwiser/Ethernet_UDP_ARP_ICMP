`timescale 1ns/1ps

// Registered-output asynchronous FIFO model for RTL throughput simulation.
// The production build uses the Intel dcfifo instance in user_dc_fifo.sv.
module user_dc_fifo_9b_1024d (
    input  wire       aclr,
    input  wire [8:0] data,
    input  wire       rdclk,
    input  wire       rdreq,
    input  wire       wrclk,
    input  wire       wrreq,
    output reg  [8:0] q,
    output wire       rdempty,
    output wire       wrfull
);
    reg [8:0] mem [0:1023];
    integer wrptr = 0;
    integer rdptr = 0;

    assign rdempty = (wrptr == rdptr);
    assign wrfull = ((wrptr - rdptr) >= 1023);

    always @(posedge wrclk or posedge aclr) begin
        if (aclr)
            wrptr <= 0;
        else if (wrreq && !wrfull) begin
            mem[wrptr & 1023] <= data;
            wrptr <= wrptr + 1;
        end
    end

    always @(posedge rdclk or posedge aclr) begin
        if (aclr) begin
            rdptr <= 0;
            q <= 9'h000;
        end else if (rdreq && !rdempty) begin
            q <= mem[rdptr & 1023];
            rdptr <= rdptr + 1;
        end
    end
endmodule

// ARP/ICMP is not traffic-generated here. This stub isolates the UDP receive
// path while retaining the production udp_axis_rx interface.
module eth_axis (
    input  wire        sys_clk,
    input  wire        sys_rst_n,
    input  wire [47:0] board_mac_addr,
    input  wire [31:0] board_ip_addr,
    axis.slave         s_axis_rx,
    axis.master        m_axis_tx,
    output wire        arp_working,
    output wire [47:0] arp_pc_mac,
    output wire [31:0] arp_pc_ip,
    output wire        arp_pc_refresh
);
    assign s_axis_rx.tready = 1'b1;
    assign m_axis_tx.tdata = 8'h00;
    assign m_axis_tx.tvalid = 1'b0;
    assign m_axis_tx.tlast = 1'b0;
    assign m_axis_tx.tuser = 1'b0;
    assign m_axis_tx.tkeep = 1'b1;
    assign m_axis_tx.tstrb = 1'b1;
    assign arp_working = 1'b0;
    assign arp_pc_mac = 48'd0;
    assign arp_pc_ip = 32'd0;
    assign arp_pc_refresh = 1'b0;
endmodule

module tb_udp_echo_saturation #(
    parameter integer RMII_PHASE_NS = 3,
    parameter integer MIN_FRAME_PACKETS = 100,
    parameter integer MTU_PACKETS = 50,
    parameter integer JUMBO_PACKETS = 100
);
    localparam [47:0] BOARD_MAC = 48'h50_12_22_33_44_55;
    localparam [31:0] BOARD_IP  = 32'h0a_0a_01_0a;
    localparam [47:0] PC_MAC    = 48'h02_11_22_33_44_55;
    localparam [31:0] PC_IP     = 32'h0a_0a_01_01;

    reg sys_clk = 1'b0;
    reg rmii_clk = 1'b0;
    reg rstn = 1'b0;
    always #10 sys_clk = ~sys_clk; // 50 MHz system clock
    initial begin
        #(RMII_PHASE_NS);
        forever #10 rmii_clk = ~rmii_clk; // 50 MHz, phase-offset CDC clock
    end

    axis rx_axis();
    axis arp_tx_axis();
    axis udp_tx_axis();
    axis phy_tx_axis();
    axis phy_rx_unused();
    pc_head rx_head();
    pc_head tx_head();

    wire udp_rxstart;
    wire udp_rxend;
    wire udp_rxframe_done;
    wire udp_rxdv;
    wire [7:0] udp_rxdata;
    wire [15:0] udp_rxamount;
    wire [15:0] udp_rxnum;
    wire udp_txstart;
    wire [15:0] udp_txamount;
    wire [7:0] udp_txdata;
    wire udp_txreq;
    wire udp_txbusy;
    wire rmii_txen;
    wire [1:0] rmii_txdata;

    reg [7:0] frame [0:8191];
    integer frame_size = 0;
    integer current_payload_len = 0;

    integer rx_done_count = 0;
    integer commit_count = 0;
    integer overflow_count = 0;
    integer meta_accept_count = 0;
    integer payload_read_count = 0;
    integer udp_axis_frame_count = 0;
    integer cdc_boundary_count = 0;
    integer output_frame_count = 0;
    integer max_data_occupancy = 0;
    integer max_committed_count = 0;
    integer max_meta_count = 0;
    integer tx_abort_count = 0;
    integer regression_failures = 0;
    reg rmii_txen_d = 1'b0;
    reg tx_abort_d = 1'b0;
    reg udp_axis_last_d = 1'b0;
    reg phy_axis_last_d = 1'b0;

    udp_axis_rx u_rx (
        .sys_clk(sys_clk),
        .sys_rst_n(rstn),
        .board_mac_addr(BOARD_MAC),
        .board_ip_addr(BOARD_IP),
        .s_axis_rx(rx_axis),
        .m_axis_tx(arp_tx_axis),
        .arp_working(),
        .test_count(),
        .udp_rxstart(udp_rxstart),
        .udp_rxend(udp_rxend),
        .udp_rxframe_done(udp_rxframe_done),
        .udp_rxdv(udp_rxdv),
        .udp_rxdata(udp_rxdata),
        .udp_rxamount(udp_rxamount),
        .udp_rxnum(udp_rxnum),
        .rx_head(rx_head)
    );

    udp_ring u_ring (
        .clk(sys_clk),
        .rstn(rstn),
        .udp_rxstart(udp_rxstart),
        .udp_rxframe_done(udp_rxframe_done),
        .udp_rxdv(udp_rxdv),
        .udp_rxdata(udp_rxdata),
        .udp_rxamount(udp_rxamount),
        .udp_rx_head(rx_head),
        .udp_txstart(udp_txstart),
        .udp_txamount(udp_txamount),
        .udp_txdata(udp_txdata),
        .udp_txreq(udp_txreq),
        .udp_txbusy(udp_txbusy),
        .udp_tx_head(tx_head)
    );

    udp_axis_tx u_tx (
        .sys_clk(sys_clk),
        .sys_rst_n(rstn),
        .board_mac_addr(BOARD_MAC),
        .board_ip_addr(BOARD_IP),
        .m_axis_tx(udp_tx_axis),
        .udp_txstart(udp_txstart),
        .udp_txamount(udp_txamount),
        .udp_txdata(udp_txdata),
        .udp_txreq(udp_txreq),
        .udp_txbusy(udp_txbusy),
        .tx_head(tx_head)
    );

    tx_cdc_fifo_axis u_tx_cdc (
        .clk(sys_clk),
        .tx_clk(rmii_clk),
        .rstn(rstn),
        .sys_tx(arp_tx_axis),
        .udp_tx(udp_tx_axis),
        .phy_tx(phy_tx_axis)
    );

    phy_rmii_axis u_phy (
        .rstn(rstn),
        .rmii_clk(rmii_clk),
        .rmii_crs_dv(1'b0),
        .rmii_rxdata(2'b00),
        .rmii_txen(rmii_txen),
        .rmii_txdata(rmii_txdata),
        .rmii_rst(),
        .m_rmii_rx_axis_net(phy_rx_unused),
        .s_rmii_tx_axis_net(phy_tx_axis)
    );

    function automatic [31:0] crc32_byte(
        input [31:0] crc_in,
        input [7:0] datum
    );
        integer bit_index;
        reg [31:0] crc;
        begin
            crc = crc_in ^ datum;
            for (bit_index = 0; bit_index < 8; bit_index = bit_index + 1)
                crc = crc[0] ? ((crc >> 1) ^ 32'hedb88320) : (crc >> 1);
            crc32_byte = crc;
        end
    endfunction

    task automatic put_byte(input [7:0] datum, inout integer index);
        begin
            frame[index] = datum;
            index = index + 1;
        end
    endtask

    task automatic build_frame(input integer payload_len);
        integer index;
        integer n;
        integer ip_start;
        integer ip_total_len;
        integer checksum_sum;
        integer checksum;
        integer ethernet_payload_len;
        reg [31:0] ethernet_crc;
        begin
            index = 0;
            for (n = 0; n < 7; n = n + 1)
                put_byte(8'h55, index);
            put_byte(8'hd5, index);

            put_byte(BOARD_MAC[47:40], index);
            put_byte(BOARD_MAC[39:32], index);
            put_byte(BOARD_MAC[31:24], index);
            put_byte(BOARD_MAC[23:16], index);
            put_byte(BOARD_MAC[15:8], index);
            put_byte(BOARD_MAC[7:0], index);
            put_byte(PC_MAC[47:40], index);
            put_byte(PC_MAC[39:32], index);
            put_byte(PC_MAC[31:24], index);
            put_byte(PC_MAC[23:16], index);
            put_byte(PC_MAC[15:8], index);
            put_byte(PC_MAC[7:0], index);
            put_byte(8'h08, index);
            put_byte(8'h00, index);

            ip_start = index;
            ip_total_len = 20 + 8 + payload_len;
            put_byte(8'h45, index);
            put_byte(8'h00, index);
            put_byte(ip_total_len[15:8], index);
            put_byte(ip_total_len[7:0], index);
            put_byte(8'h12, index);
            put_byte(8'h34, index);
            put_byte(8'h00, index); // no IPv4 fragmentation
            put_byte(8'h00, index);
            put_byte(8'h40, index);
            put_byte(8'h11, index);
            put_byte(8'h00, index);
            put_byte(8'h00, index);
            put_byte(PC_IP[31:24], index);
            put_byte(PC_IP[23:16], index);
            put_byte(PC_IP[15:8], index);
            put_byte(PC_IP[7:0], index);
            put_byte(BOARD_IP[31:24], index);
            put_byte(BOARD_IP[23:16], index);
            put_byte(BOARD_IP[15:8], index);
            put_byte(BOARD_IP[7:0], index);

            checksum_sum = 0;
            for (n = 0; n < 20; n = n + 2)
                checksum_sum = checksum_sum + {frame[ip_start+n], frame[ip_start+n+1]};
            while (checksum_sum > 16'hffff)
                checksum_sum = (checksum_sum & 16'hffff) + (checksum_sum >> 16);
            checksum = (~checksum_sum) & 16'hffff;
            frame[ip_start+10] = checksum[15:8];
            frame[ip_start+11] = checksum[7:0];

            put_byte(8'h11, index);
            put_byte(8'h11, index);
            put_byte(8'h22, index);
            put_byte(8'h22, index);
            put_byte((payload_len+8) >> 8, index);
            put_byte((payload_len+8) & 8'hff, index);
            put_byte(8'h00, index);
            put_byte(8'h00, index);
            for (n = 0; n < payload_len; n = n + 1)
                put_byte((n * 8'h29 + 8'h63) & 8'hff, index);

            ethernet_payload_len = ip_total_len;
            while (ethernet_payload_len < 46) begin
                put_byte(8'h00, index);
                ethernet_payload_len = ethernet_payload_len + 1;
            end

            ethernet_crc = 32'hffff_ffff;
            for (n = 8; n < index; n = n + 1)
                ethernet_crc = crc32_byte(ethernet_crc, frame[n]);
            ethernet_crc = ~ethernet_crc;
            put_byte(ethernet_crc[7:0], index);
            put_byte(ethernet_crc[15:8], index);
            put_byte(ethernet_crc[23:16], index);
            put_byte(ethernet_crc[31:24], index);
            frame_size = index;
            current_payload_len = payload_len;
        end
    endtask

    // One byte per four 50 MHz clocks models 100 Mb/s RMII. tlast is the
    // frame-level signal used by this project. The 48-clock gap is the minimum
    // 96-bit Ethernet inter-packet gap.
    task automatic transmit_frame_at_line_rate;
        integer n;
        begin
            rx_axis.tlast = 1'b1;
            for (n = 0; n < frame_size; n = n + 1) begin
                rx_axis.tvalid = 1'b1;
                rx_axis.tdata = frame[n];
                @(negedge sys_clk);
                rx_axis.tvalid = 1'b0;
                repeat (3) @(negedge sys_clk);
            end
            rx_axis.tlast = 1'b0;
            rx_axis.tdata = 8'h00;
            repeat (48) @(negedge sys_clk);
        end
    endtask

    task automatic reset_case;
        begin
            rstn = 1'b0;
            rx_axis.tvalid = 1'b0;
            rx_axis.tlast = 1'b0;
            rx_axis.tdata = 8'h00;
            repeat (8) @(posedge sys_clk);
            rstn = 1'b1;
            repeat (8) @(posedge sys_clk);
            @(negedge sys_clk);
        end
    endtask

    task automatic wait_until_drained(input integer expected_rx_frames);
        integer timeout_cycles;
        begin
            timeout_cycles = 0;
            while ((meta_accept_count < commit_count ||
                    u_ring.committed_count != 0 || u_ring.meta_count != 0 ||
                    udp_txbusy || !u_tx_cdc.udp_empty ||
                    u_tx_cdc.udp_marker_pending || u_tx_cdc.rd_pending ||
                    u_tx_cdc.in_frame || u_tx_cdc.out_valid ||
                    u_phy.tx_buf_valid || rmii_txen) &&
                   timeout_cycles < 2000000) begin
                @(posedge sys_clk);
                timeout_cycles = timeout_cycles + 1;
            end
            repeat (200) @(posedge sys_clk);
            if (timeout_cycles >= 2000000)
                $fatal(1, "SATURATION_DRAIN_TIMEOUT payload=%0d rx=%0d commit=%0d accepted=%0d",
                       current_payload_len, expected_rx_frames, commit_count,
                       meta_accept_count);
        end
    endtask

    task automatic report_case(
        input [127:0] case_name,
        input integer payload_len,
        input integer packet_count
    );
        integer packet_index;
        begin
            build_frame(payload_len);
            reset_case();
            for (packet_index = 0; packet_index < packet_count;
                 packet_index = packet_index + 1)
                transmit_frame_at_line_rate();
            wait_until_drained(packet_count);
            $display("SATURATION_CASE phase_ns=%0d name=%0s payload=%0d sent=%0d rx_valid=%0d committed=%0d dropped=%0d overflows=%0d tx_packets=%0d tx_axis_frames=%0d cdc_boundaries=%0d rmii_frames=%0d payload_reads=%0d max_data=%0d max_committed=%0d max_meta=%0d tx_aborts=%0d",
                     RMII_PHASE_NS, case_name, payload_len, packet_count, rx_done_count,
                     commit_count, rx_done_count-commit_count, overflow_count,
                     meta_accept_count, udp_axis_frame_count, cdc_boundary_count,
                     output_frame_count, payload_read_count, max_data_occupancy,
                     max_committed_count, max_meta_count, tx_abort_count);
            if (udp_axis_frame_count != cdc_boundary_count ||
                cdc_boundary_count != output_frame_count ||
                tx_abort_count != 0 || meta_accept_count != commit_count ||
                payload_read_count != commit_count * payload_len) begin
                $display("SATURATION_BOUNDARY_FAIL phase_ns=%0d name=%0s axis=%0d cdc=%0d rmii=%0d aborts=%0d commits=%0d accepted=%0d reads=%0d/%0d",
                         RMII_PHASE_NS, case_name, udp_axis_frame_count, cdc_boundary_count,
                         output_frame_count, tx_abort_count, commit_count,
                         meta_accept_count, payload_read_count,
                         commit_count * payload_len);
                regression_failures = regression_failures + 1;
            end
        end
    endtask

    always @(posedge sys_clk) begin
        if (!rstn) begin
            rx_done_count = 0;
            commit_count = 0;
            overflow_count = 0;
            meta_accept_count = 0;
            payload_read_count = 0;
            udp_axis_frame_count = 0;
            udp_axis_last_d = 1'b0;
            max_data_occupancy = 0;
            max_committed_count = 0;
            max_meta_count = 0;
        end else begin
            if (udp_rxframe_done)
                rx_done_count = rx_done_count + 1;
            if (u_ring.packet_commit)
                commit_count = commit_count + 1;
            if (u_ring.data_overflow)
                overflow_count = overflow_count + 1;
            if (u_ring.meta_read)
                meta_accept_count = meta_accept_count + 1;
            if (udp_txreq)
                payload_read_count = payload_read_count + 1;
            if (udp_axis_last_d && !udp_tx_axis.tlast)
                udp_axis_frame_count = udp_axis_frame_count + 1;
            udp_axis_last_d = udp_tx_axis.tlast;
            if ((u_ring.committed_count + u_ring.stage_count) > max_data_occupancy)
                max_data_occupancy = u_ring.committed_count + u_ring.stage_count;
            if (u_ring.committed_count > max_committed_count)
                max_committed_count = u_ring.committed_count;
            if (u_ring.meta_count > max_meta_count)
                max_meta_count = u_ring.meta_count;
        end
    end

    always @(posedge rmii_clk) begin
        if (!rstn) begin
            rmii_txen_d = 1'b0;
            tx_abort_d = 1'b0;
            phy_axis_last_d = 1'b0;
            cdc_boundary_count = 0;
            output_frame_count = 0;
            tx_abort_count = 0;
        end else begin
            if (rmii_txen && !rmii_txen_d)
                output_frame_count = output_frame_count + 1;
            if (u_phy.tx_abort && !tx_abort_d)
                tx_abort_count = tx_abort_count + 1;
            if (phy_axis_last_d && !phy_tx_axis.tlast)
                cdc_boundary_count = cdc_boundary_count + 1;
            rmii_txen_d <= rmii_txen;
            tx_abort_d <= u_phy.tx_abort;
            phy_axis_last_d <= phy_tx_axis.tlast;
        end
    end

    initial begin
        rx_axis.tvalid = 1'b0;
        rx_axis.tlast = 1'b0;
        rx_axis.tdata = 8'h00;
        rx_axis.tuser = 1'b0;
        rx_axis.tkeep = 1'b1;
        rx_axis.tstrb = 1'b1;

        report_case("min_frame", 12, MIN_FRAME_PACKETS);
        report_case("mtu_1472", 1472, MTU_PACKETS);
        report_case("jumbo_1473", 1473, JUMBO_PACKETS);

        if (regression_failures == 0)
            $display("SATURATION_REGRESSION_PASS phase_ns=%0d boundary_cases=3",
                     RMII_PHASE_NS);
        else
            $fatal(1, "SATURATION_REGRESSION_FAIL failures=%0d",
                   regression_failures);
        $finish;
    end

    initial begin
        #1000000000;
        $fatal(1, "SATURATION_GLOBAL_TIMEOUT");
    end
endmodule
