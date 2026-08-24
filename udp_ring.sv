`include "pc_head.svh"

// Packet-transactional UDP loopback buffer. RX bytes remain staged until the
// complete Ethernet frame (including FCS) has been validated. An overflow or
// rejected frame rolls back only the packet currently being received.
module udp_ring #(
	parameter DATA_FIFO_DEPTH = 2048,
	parameter META_FIFO_DEPTH = 64
)(
	input  wire                     clk,
	input  wire                     rstn,

	input  wire                     udp_rxstart,
	input  wire                     udp_rxframe_done,
	input  wire                     udp_rxdv,
	input  wire [7:0]               udp_rxdata,
	input  wire [15:0]              udp_rxamount,
	pc_head.slave                   udp_rx_head,

	output wire                     udp_txstart,
	output wire [15:0]              udp_txamount,
	output wire [7:0]               udp_txdata,
	input  wire                     udp_txreq,
	input  wire                     udp_txbusy,
	pc_head.master                  udp_tx_head
);

	localparam DATA_PTR_W = (DATA_FIFO_DEPTH <= 2) ? 1 : $clog2(DATA_FIFO_DEPTH);
	localparam META_PTR_W = (META_FIFO_DEPTH <= 2) ? 1 : $clog2(META_FIFO_DEPTH);
	localparam DATA_CNT_W = $clog2(DATA_FIFO_DEPTH + 1);
	localparam META_CNT_W = $clog2(META_FIFO_DEPTH + 1);

	logic [7:0]   data_mem [0:DATA_FIFO_DEPTH-1];
	logic [127:0] meta_mem [0:META_FIFO_DEPTH-1];
	logic [DATA_PTR_W-1:0] data_rd_ptr;
	logic [DATA_PTR_W-1:0] data_commit_ptr;
	logic [DATA_PTR_W-1:0] data_stage_ptr;
	logic [META_PTR_W-1:0] meta_rd_ptr;
	logic [META_PTR_W-1:0] meta_wr_ptr;
	logic [DATA_CNT_W-1:0] committed_count;
	logic [DATA_CNT_W-1:0] stage_count;
	logic [META_CNT_W-1:0] meta_count;
	logic packet_active;
	logic rx_drop;

	wire [127:0] rx_meta_data = {
		udp_rx_head.pc_mac_addr,
		udp_rx_head.pc_ip_addr,
		udp_rx_head.pc_port,
		udp_rx_head.board_port,
		udp_rxamount
	};
	wire [127:0] tx_meta_data = meta_mem[meta_rd_ptr];
	wire data_read = udp_txreq && (committed_count != 0);
	wire meta_read = udp_txstart && !udp_txbusy;
	wire data_space = (committed_count + stage_count < DATA_FIFO_DEPTH) || data_read;
	wire data_write = packet_active && udp_rxdv && !rx_drop && data_space;
	wire data_overflow = packet_active && udp_rxdv && !rx_drop && !data_space;
	wire meta_space = (meta_count < META_FIFO_DEPTH) || meta_read;
	wire packet_commit = udp_rxframe_done && packet_active && !rx_drop &&
		!data_overflow && (stage_count == udp_rxamount) && meta_space;

	function automatic [DATA_PTR_W-1:0] data_ptr_next(
		input [DATA_PTR_W-1:0] ptr
	);
		begin
			if (ptr == DATA_FIFO_DEPTH-1)
				data_ptr_next = {DATA_PTR_W{1'b0}};
			else
				data_ptr_next = ptr + 1'b1;
		end
	endfunction

	function automatic [META_PTR_W-1:0] meta_ptr_next(
		input [META_PTR_W-1:0] ptr
	);
		begin
			if (ptr == META_FIFO_DEPTH-1)
				meta_ptr_next = {META_PTR_W{1'b0}};
			else
				meta_ptr_next = ptr + 1'b1;
		end
	endfunction

	assign udp_txstart = (meta_count != 0);
	assign udp_txdata = data_mem[data_rd_ptr];
	assign {
		udp_tx_head.pc_mac_addr,
		udp_tx_head.pc_ip_addr,
		udp_tx_head.pc_port,
		udp_tx_head.board_port,
		udp_txamount
	} = tx_meta_data;

	always_ff @(posedge clk or negedge rstn) begin
		if (!rstn) begin
			data_rd_ptr <= '0;
			data_commit_ptr <= '0;
			data_stage_ptr <= '0;
			meta_rd_ptr <= '0;
			meta_wr_ptr <= '0;
			committed_count <= '0;
			stage_count <= '0;
			meta_count <= '0;
			packet_active <= 1'b0;
			rx_drop <= 1'b0;
		end else begin
			if (data_read)
				data_rd_ptr <= data_ptr_next(data_rd_ptr);

			if (meta_read)
				meta_rd_ptr <= meta_ptr_next(meta_rd_ptr);

			if (udp_rxstart) begin
				// Also abandons a previous frame that never received a valid FCS.
				data_stage_ptr <= data_commit_ptr;
				stage_count <= '0;
				packet_active <= 1'b1;
				rx_drop <= 1'b0;
			end else begin
				if (data_write) begin
					data_mem[data_stage_ptr] <= udp_rxdata;
					data_stage_ptr <= data_ptr_next(data_stage_ptr);
					stage_count <= stage_count + 1'b1;
				end
				if (data_overflow)
					rx_drop <= 1'b1;

				if (udp_rxframe_done) begin
					packet_active <= 1'b0;
					rx_drop <= 1'b0;
					stage_count <= '0;
					if (packet_commit) begin
						data_commit_ptr <= data_stage_ptr;
						meta_mem[meta_wr_ptr] <= rx_meta_data;
						meta_wr_ptr <= meta_ptr_next(meta_wr_ptr);
					end else begin
						data_stage_ptr <= data_commit_ptr;
					end
				end
			end

			case ({packet_commit, data_read})
				2'b10: committed_count <= committed_count + stage_count;
				2'b01: committed_count <= committed_count - 1'b1;
				2'b11: committed_count <= committed_count + stage_count - 1'b1;
				default: committed_count <= committed_count;
			endcase

			case ({packet_commit, meta_read})
				2'b10: meta_count <= meta_count + 1'b1;
				2'b01: meta_count <= meta_count - 1'b1;
				default: meta_count <= meta_count;
			endcase
		end
	end

endmodule
