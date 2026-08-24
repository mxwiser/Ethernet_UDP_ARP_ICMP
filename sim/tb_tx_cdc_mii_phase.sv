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

module tb_tx_cdc_mii_phase #(
    parameter integer MII_PHASE_NS = 0,
    parameter integer MII_HALF_PERIOD_PS = 20000,
    parameter integer FRAME_COUNT = 256
);
    reg sys_clk = 1'b0;
    reg mii_clk = 1'b0;
    reg rstn = 1'b0;
    always #10000 sys_clk = ~sys_clk; // 50 MHz
    initial begin
        #(MII_PHASE_NS * 1000);
        forever #(MII_HALF_PERIOD_PS) mii_clk = ~mii_clk; // 25 MHz nominal
    end

    axis sys_tx();
    axis udp_tx();
    axis phy_tx();
    axis phy_rx_unused();

    wire [3:0] mii_txd;
    wire mii_txen;

    integer source_frame;
    integer output_frame = 0;
    integer output_byte = 0;
    integer output_nibble = 0;
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
        .tx_clk(mii_clk),
        .rstn(rstn),
        .sys_tx(sys_tx),
        .udp_tx(udp_tx),
        .phy_tx(phy_tx)
    );

    phy_mii_axis #(
        .TX_IFG_CLKS(24)
    ) u_phy (
        .rstn(rstn),
        .tx_clk(mii_clk),
        .txd(mii_txd),
        .txen(mii_txen),
        .rx_clk(mii_clk),
        .rxd(4'h0),
        .rxdv(1'b0),
        .m_phy_rx(phy_rx_unused),
        .s_phy_tx(phy_tx)
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

    always @(posedge mii_clk) begin
        if (!rstn) begin
            output_frame = 0;
            output_byte = 0;
            output_nibble = 0;
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

            if (!mii_txen && waiting_for_next_frame)
                ifg_clocks = ifg_clocks + 1;

            if (mii_txen && !txen_d) begin
                if (waiting_for_next_frame && ifg_clocks < 24) begin
                    $display("MII_IFG_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d clocks=%0d",
                             MII_PHASE_NS, MII_HALF_PERIOD_PS,
                             output_frame, ifg_clocks);
                    failures = failures + 1;
                end
                waiting_for_next_frame = 1'b0;
            end

            if (mii_txen) begin
                if (output_nibble == 0) begin
                    assembled_byte[3:0] = mii_txd;
                    output_nibble = 1;
                end else begin
                    assembled_byte[7:4] = mii_txd;
                    if (output_frame >= FRAME_COUNT ||
                        assembled_byte !== test_byte(output_frame, output_byte)) begin
                        $display("MII_DATA_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d byte=%0d got=%02x expected=%02x",
                                 MII_PHASE_NS, MII_HALF_PERIOD_PS,
                                 output_frame, output_byte, assembled_byte,
                                 test_byte(output_frame, output_byte));
                        failures = failures + 1;
                    end
                    output_byte = output_byte + 1;
                    output_nibble = 0;
                end
            end

            if (!mii_txen && txen_d) begin
                if (output_nibble != 0 ||
                    output_byte != test_frame_length(output_frame)) begin
                    $display("MII_LENGTH_FAIL phase_ns=%0d half_period_ps=%0d frame=%0d bytes=%0d/%0d nibble=%0d",
                             MII_PHASE_NS, MII_HALF_PERIOD_PS,
                             output_frame, output_byte,
                             test_frame_length(output_frame), output_nibble);
                    failures = failures + 1;
                end
                output_frame = output_frame + 1;
                output_byte = 0;
                output_nibble = 0;
                ifg_clocks = 0;
                waiting_for_next_frame = 1'b1;
            end

            txen_d = mii_txen;
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
        repeat (20) @(posedge mii_clk);

        if (cdc_boundaries != FRAME_COUNT || output_frame != FRAME_COUNT ||
            tx_aborts != 0 || failures != 0)
            $fatal(1, "MII_PHASE_SWEEP_FAIL phase_ns=%0d half_period_ps=%0d source=%0d cdc=%0d mii=%0d aborts=%0d failures=%0d",
                   MII_PHASE_NS, MII_HALF_PERIOD_PS, FRAME_COUNT,
                   cdc_boundaries, output_frame, tx_aborts, failures);
        else
            $display("MII_PHASE_SWEEP_PASS phase_ns=%0d half_period_ps=%0d frames=%0d cdc=%0d mii=%0d aborts=%0d",
                     MII_PHASE_NS, MII_HALF_PERIOD_PS, FRAME_COUNT,
                     cdc_boundaries, output_frame, tx_aborts);
        $finish;
    end

    initial begin
        #100000000000;
        $fatal(1, "MII_PHASE_SWEEP_TIMEOUT phase_ns=%0d half_period_ps=%0d source=%0d cdc=%0d mii=%0d",
               MII_PHASE_NS, MII_HALF_PERIOD_PS, source_frame,
               cdc_boundaries, output_frame);
    end
endmodule
