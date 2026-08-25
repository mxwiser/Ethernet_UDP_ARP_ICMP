module command_fifo #(
    parameter integer DATA_WIDTH = 64,
    parameter integer DEPTH      = 16
)(
    input  wire                   clk,
    input  wire                   rstn,
    input  wire                   clear,
    input  wire                   wr_en,
    input  wire  [DATA_WIDTH-1:0] wr_data,
    input  wire                   rd_en,
    output wire  [DATA_WIDTH-1:0] rd_data,
    output wire                   empty,
    output wire                   full
);

localparam integer PTR_WIDTH = (DEPTH <= 2) ? 1 : $clog2(DEPTH);

logic [DATA_WIDTH-1:0] memory [0:DEPTH-1];
logic [PTR_WIDTH-1:0] write_pointer;
logic [PTR_WIDTH-1:0] read_pointer;
logic [PTR_WIDTH:0]   item_count;

wire write_accepted = wr_en && !full;
wire read_accepted  = rd_en && !empty;

assign rd_data = memory[read_pointer];
assign empty   = (item_count == 0);
assign full    = (item_count == DEPTH);

always_ff @(posedge clk or negedge rstn) begin
    if (!rstn) begin
        write_pointer <= '0;
        read_pointer  <= '0;
        item_count    <= '0;
    end else if (clear) begin
        write_pointer <= '0;
        read_pointer  <= '0;
        item_count    <= '0;
    end else begin
        if (write_accepted) begin
            memory[write_pointer] <= wr_data;
            if (write_pointer == DEPTH - 1)
                write_pointer <= '0;
            else
                write_pointer <= write_pointer + 1'b1;
        end

        if (read_accepted) begin
            if (read_pointer == DEPTH - 1)
                read_pointer <= '0;
            else
                read_pointer <= read_pointer + 1'b1;
        end

        case ({write_accepted, read_accepted})
            2'b10: item_count <= item_count + 1'b1;
            2'b01: item_count <= item_count - 1'b1;
            default: item_count <= item_count;
        endcase
    end
end

endmodule
