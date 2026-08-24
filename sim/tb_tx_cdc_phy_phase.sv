`timescale 1ps/1ps

// Registered-output simulation model matching the Intel dcfifo interface.
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

module tb_tx_cdc_phy_phase #(
    parameter integer RMII_PHASE_NS = 0,
    parameter integer RMII_HALF_PERIOD_PS = 10000,
    parameter integer FRAME_COUNT = 256
);
    reg sys_clk = 1'b0;
    reg rmii_clk = 1'b0;
    reg rstn = 1'b0;
    always #10000 sys_clk = ~sys_clk;
    initial begin
        #(RMII_PHASE_NS * 1000);
        forever #(RMII_HALF_PERIOD_PS) rmii_clk = ~rmii_clk;
    end

    axis sys_tx();
    axis udp_tx();
    axis phy_tx();
    axis phy_rx_unused();

    wire rmii_txen;
    wire [1:0] rmii_txdata;

    integer source_frame;
    integer output_frame = 0;
    integer output_byte = 0;
    integer output_dibit = 0;
    integer cdc_boundaries = 0;
    integer tx_aborts = 0;
    integer ifg_clocks = 0;
    integer failures = 0;
    reg [7:0] assembled_byte = 8'h00;
    reg txen_d = 1'b0;
    reg phy_last_d = 1'b0;
    reg tx_abort_d = 1'b0;
    reg waiting_for_next_frame = 1'b0;

    function automatic integer test_frame_length(input integer frame_number);
        begin
            test_frame_length = 18 + ((frame_number * 37) % 96);
        end
    endfunction

    function automatic [7:0] test_byte(
        input integer frame_number,
        input integer byte_number
    );
        begin
            test_byte = (frame_number * 8'h53 +
                         byte_number * 8'h29 + 8'h17) & 8'hff;
        end
    endfunction

    tx_cdc_fifo_axis u_cdc (
        .clk(sys_clk),
        .tx_clk(rmii_clk),
        .rstn(rstn),
        .sys_tx(sys_tx),
        .udp_tx(udp_tx),
        .phy_tx(phy_tx)
    );

    phy_rmii_axis #(
        .TX_IFG_CLKS(48)
    ) u_phy (
        .rstn(rstn),
        .rmii_clk(rmii_clk),
        .rmii_crs_dv(1'b0),
        .rmii_rxdata(2'b00),
        .rmii_txen(rmii_txen),
        .rmii_txdata(rmii_txdata),
        .rmii_rst(),
        .m_rmii_rx_axis_net(phy_rx_unused),
        .s_rmii_tx_axis_net(phy_tx)
    );

    task automatic send_frame(input integer frame_number);
        integer length;
        integer index;
        begin
            length = test_frame_length(frame_number);
            index = 0;
            @(negedge sys_clk);
            udp_tx.tvalid = 1'b1;
            udp_tx.tlast = 1'b1;
            udp_tx.tdata = test_byte(frame_number, 0);
            while (index < length) begin
                @(posedge sys_clk);
                if (udp_tx.tready) begin
                    index = index + 1;
                    @(negedge sys_clk);
                    if (index < length)
                        udp_tx.tdata = test_byte(frame_number, index);
                    else begin
                        udp_tx.tvalid = 1'b0;
                        udp_tx.tlast = 1'b0;
                    end
                end
            end
        end
    endtask

    always @(posedge rmii_clk) begin
        if (!rstn) begin
            output_frame = 0;
            output_byte = 0;
            output_dibit = 0;
            cdc_boundaries = 0;
            tx_aborts = 0;
            ifg_clocks = 0;
            failures = 0;
            assembled_byte = 8'h00;
            txen_d = 1'b0;
            phy_last_d = 1'b0;
            tx_abort_d = 1'b0;
            waiting_for_next_frame = 1'b0;
        end else begin
            if (phy_last_d && !phy_tx.tlast)
                cdc_boundaries = cdc_boundaries + 1;

            if (u_phy.tx_abort && !tx_abort_d)
                tx_aborts = tx_aborts + 1;

            if (!rmii_txen && waiting_for_next_frame)
                ifg_clocks = ifg_clocks + 1;

            if (rmii_txen && !txen_d) begin
                if (waiting_for_next_frame && ifg_clocks < 48) begin
                    $display("PHASE_IFG_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d clocks=%0d",
                             RMII_PHASE_NS, RMII_HALF_PERIOD_PS,
                             output_frame, ifg_clocks);
                    failures = failures + 1;
                end
                waiting_for_next_frame = 1'b0;
            end

            if (rmii_txen) begin
                case (output_dibit)
                    0: assembled_byte[1:0] = rmii_txdata;
                    1: assembled_byte[3:2] = rmii_txdata;
                    2: assembled_byte[5:4] = rmii_txdata;
                    3: begin
                        assembled_byte[7:6] = rmii_txdata;
                        if (output_frame >= FRAME_COUNT ||
                            assembled_byte !== test_byte(output_frame, output_byte)) begin
                            $display("PHASE_DATA_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d byte=%0d got=%02x expected=%02x",
                                     RMII_PHASE_NS, RMII_HALF_PERIOD_PS,
                                     output_frame, output_byte,
                                     assembled_byte,
                                     test_byte(output_frame, output_byte));
                            failures = failures + 1;
                        end
                        output_byte = output_byte + 1;
                    end
                endcase
                output_dibit = (output_dibit == 3) ? 0 : output_dibit + 1;
            end

            if (!rmii_txen && txen_d) begin
                if (output_dibit != 0 ||
                    output_byte != test_frame_length(output_frame)) begin
                    $display("PHASE_LENGTH_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d bytes=%0d/%0d dibit=%0d",
                             RMII_PHASE_NS, RMII_HALF_PERIOD_PS,
                             output_frame, output_byte,
                             test_frame_length(output_frame), output_dibit);
                    failures = failures + 1;
                end
                output_frame = output_frame + 1;
                output_byte = 0;
                output_dibit = 0;
                ifg_clocks = 0;
                waiting_for_next_frame = 1'b1;
            end

            txen_d = rmii_txen;
            phy_last_d = phy_tx.tlast;
            tx_abort_d = u_phy.tx_abort;
        end
    end

    initial begin
        sys_tx.tvalid = 1'b0;
        sys_tx.tlast = 1'b0;
        sys_tx.tdata = 8'h00;
        sys_tx.tuser = 1'b0;
        sys_tx.tkeep = 1'b1;
        sys_tx.tstrb = 1'b1;
        udp_tx.tvalid = 1'b0;
        udp_tx.tlast = 1'b0;
        udp_tx.tdata = 8'h00;
        udp_tx.tuser = 1'b0;
        udp_tx.tkeep = 1'b1;
        udp_tx.tstrb = 1'b1;

        repeat (8) @(posedge sys_clk);
        rstn = 1'b1;
        repeat (8) @(posedge sys_clk);

        for (source_frame = 0; source_frame < FRAME_COUNT;
             source_frame = source_frame + 1)
            send_frame(source_frame);

        wait (output_frame == FRAME_COUNT);
        repeat (20) @(posedge rmii_clk);

        if (cdc_boundaries != FRAME_COUNT || output_frame != FRAME_COUNT ||
            tx_aborts != 0 || failures != 0)
            $fatal(1, "PHASE_SWEEP_FAIL phase_ns=%0d half_period_ps=%0d source=%0d cdc=%0d rmii=%0d aborts=%0d failures=%0d",
                   RMII_PHASE_NS, RMII_HALF_PERIOD_PS, FRAME_COUNT,
                   cdc_boundaries, output_frame,
                   tx_aborts, failures);
        else
            $display("PHASE_SWEEP_PASS phase_ns=%0d half_period_ps=%0d frames=%0d cdc=%0d rmii=%0d aborts=%0d",
                     RMII_PHASE_NS, RMII_HALF_PERIOD_PS, FRAME_COUNT,
                     cdc_boundaries,
                     output_frame, tx_aborts);
        $finish;
    end

    initial begin
        #100000000000;
        $fatal(1, "PHASE_SWEEP_TIMEOUT phase_ns=%0d half_period_ps=%0d source=%0d cdc=%0d rmii=%0d",
               RMII_PHASE_NS, RMII_HALF_PERIOD_PS, source_frame,
               cdc_boundaries, output_frame);
    end
endmodule
