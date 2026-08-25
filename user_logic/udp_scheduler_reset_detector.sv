module udp_scheduler_reset_detector (
    input  wire        clk,
    input  wire        rstn,
    input  wire        udp_rxstart,
    input  wire        udp_rxend,
    input  wire        udp_rxframe_done,
    input  wire        udp_rxdv,
    input  wire [7:0]  udp_rxdata,
    input  wire [15:0] udp_rxamount,
    output logic       reset_pulse
);

logic        receiving;
logic [2:0]  byte_index;
logic [7:0]  frame_header;
logic [7:0]  function_code;
logic [31:0] received_crc;
logic [31:0] calculated_crc;
logic        pending_reset;

logic        crc_start;
logic        crc_enable;
logic        crc_end;
wire [31:0] crc32_value;
wire        crc32_valid;

CRC32_D8 u_reset_crc32 (
    .sys_clk     (clk),
    .sys_rst_n   (rstn),
    .data        (udp_rxdata),
    .crc_start   (crc_start),
    .crc_en      (crc_enable),
    .crc_end     (crc_end),
    .crc32       (crc32_value),
    .crc32_valid (crc32_valid)
);

// This detector observes the UDP stream independently of command-parser
// backpressure. A reset can therefore be recognized while a normal packet is
// waiting behind a full scheduler. CRC covers only FF 03; FCS is checked by
// requiring udp_rxframe_done before reset_pulse is emitted.
always_comb begin
    crc_start  = 1'b0;
    crc_enable = 1'b0;
    crc_end    = 1'b0;

    if (udp_rxdv) begin
        if (udp_rxstart || (receiving && (byte_index == 0))) begin
            crc_start  = 1'b1;
            crc_enable = 1'b1;
        end else if (receiving && (byte_index == 1)) begin
            crc_enable = 1'b1;
            crc_end    = (udp_rxdata == 8'h03);
        end
    end
end

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        receiving      <= 1'b0;
        byte_index     <= '0;
        frame_header   <= '0;
        function_code  <= '0;
        received_crc   <= '0;
        calculated_crc <= '0;
        pending_reset  <= 1'b0;
        reset_pulse    <= 1'b0;
    end else begin
        reset_pulse <= 1'b0;

        if (udp_rxstart) begin
            receiving      <= 1'b1;
            byte_index     <= udp_rxdv ? 3'd1 : 3'd0;
            frame_header   <= udp_rxdv ? udp_rxdata : 8'd0;
            function_code  <= '0;
            received_crc   <= '0;
            calculated_crc <= '0;
            pending_reset  <= 1'b0;
        end else if (receiving && udp_rxdv) begin
            byte_index <= byte_index + 1'b1;

            if (byte_index == 0)
                frame_header <= udp_rxdata;
            else if (byte_index == 1)
                function_code <= udp_rxdata;

            if ((function_code == 8'h03) &&
                (byte_index >= 2) && (byte_index <= 5)) begin
                received_crc <= {received_crc[23:0], udp_rxdata};
            end

            if (crc32_valid)
                calculated_crc <= crc32_value;

            if (udp_rxend) begin
                receiving <= 1'b0;
                if ((frame_header == 8'hFF) &&
                    (function_code == 8'h03) &&
                    (udp_rxamount == 16'd6) &&
                    (byte_index == 3'd5) &&
                    (calculated_crc ==
                        {received_crc[23:0], udp_rxdata})) begin
                    pending_reset <= 1'b1;
                end else begin
                    pending_reset <= 1'b0;
                end
            end
        end

        if (udp_rxframe_done) begin
            if (pending_reset)
                reset_pulse <= 1'b1;
            pending_reset <= 1'b0;
        end
    end
end

endmodule
