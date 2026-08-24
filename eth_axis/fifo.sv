// 同步 FIFO，可综合成 M9K
module fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 4096
)
(
    input  wire                  clock,
    input  wire                  rstn,
    input  wire                  clear,          // 同步清空: 复位读写指针(单拍拉高即可)
    input  wire [DATA_WIDTH-1:0] data,
    input  wire                  wrreq,
    input  wire                  rdreq,
    output wire                  empty,
    output wire                  full,
    output logic [DATA_WIDTH-1:0] q
);


localparam ADDR_WIDTH = $clog2(DEPTH);


//--------------------------------------------------
// RAM
//--------------------------------------------------

reg [DATA_WIDTH-1:0] mem [DEPTH-1:0];

//--------------------------------------------------
// pointer
//--------------------------------------------------
// 多1位用于判断full
//--------------------------------------------------
reg [ADDR_WIDTH:0] wr_ptr;
reg [ADDR_WIDTH:0] rd_ptr;
//--------------------------------------------------
// FIFO RAM
// Simple Dual Port RAM
//--------------------------------------------------
always @(posedge clock)
begin

    // write port
    if(wrreq && !full)begin
        mem[wr_ptr] <= data;
    end
end

// 读数据直接组合输出(读指针与读数据同拍), 支持背靠背连续读
always_comb begin 
    q = (rdreq && !empty) ? mem[rd_ptr] : {DATA_WIDTH{1'b0}};
end
//--------------------------------------------------
// pointer control
//--------------------------------------------------
always @(posedge clock or negedge rstn)
begin

    if(!rstn)
    begin
        wr_ptr <= 0;
        rd_ptr <= 0;
    end else if(clear) begin
        wr_ptr <= 0;
        rd_ptr <= 0;
    end else begin

        if(wrreq && !full)begin
            wr_ptr <= (wr_ptr + 1'b1)%DEPTH;
        end


        if(rdreq && !empty)begin
            rd_ptr <= (rd_ptr + 1'b1)%DEPTH;
        end

    end

end
//--------------------------------------------------
// status
//--------------------------------------------------
wire [ADDR_WIDTH:0] count;
assign count = (rd_ptr+DEPTH -wr_ptr)%DEPTH;
assign empty = (wr_ptr == rd_ptr);


assign full = (count==1);



endmodule
