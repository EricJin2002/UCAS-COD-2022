`timescale 10ns / 1ns
//`include "sources/custom_cpu/cache/tag_array.v"
//`include "sources/custom_cpu/cache/data_array.v"
`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module icache_top (
	input	      clk,
	input	      rst,
	
	//CPU interface
	/** CPU instruction fetch request to Cache: valid signal */
	input         from_cpu_inst_req_valid,
	/** CPU instruction fetch request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_inst_req_addr,
	/** Acknowledgement from Cache: ready to receive CPU instruction fetch request */
	output        to_cpu_inst_req_ready,
	
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit Instruction value */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive Instruction */
	input	      from_cpu_cache_rsp_ready,

	//Memory interface (32 byte aligned address)
	/** Cache sending memory read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address (32 byte alignment) */
	output [31:0] to_mem_rd_req_addr,
	/** Acknowledgement from memory: ready to receive memory read request */
	input         from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input         from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input         from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready
);

//TODO: Please add your I-Cache code here

	reg						  	valid_array	[`CACHE_WAY - 1:0][`CACHE_SET - 1:0];

	wire  			    	 	wen			[`CACHE_WAY - 1:0];
	wire [	 `TAG_LEN - 1:0]	tag_rdata	[`CACHE_WAY - 1:0];
	wire [	`LINE_LEN - 1:0]	data_rdata	[`CACHE_WAY - 1:0];
	
	wire 						tag_hit		[`CACHE_WAY - 1:0];
	wire [				1:0] 	way_selected;
	wire [	`LINE_LEN - 1:0]	data_selected;
	reg  [				3:0]	LRU_cnt		[`CACHE_WAY - 1:0][`CACHE_SET - 1:0];
	wire [				1:0]	way_LRU		[`CACHE_SET - 1:0];
	reg	 [			   31:0]	data_recv	[			  7:0];
	reg  [				2:0]	len;

	wire [23:0] tag;
	wire [ 2:0] index;
	wire [ 4:0] offset;

	reg [7:0] current_state;
	reg [7:0] next_state;

	localparam WAIT		= 8'b00000001;
	localparam TAG_RD	= 8'b00000010;
	localparam CACHE_RD	= 8'b00000100;
	localparam RESP		= 8'b00001000;
	localparam EVICT	= 8'b00010000;
	localparam MEM_RD	= 8'b00100000;
	localparam RECV		= 8'b01000000;
	localparam REFILL	= 8'b10000000;

	localparam isWAIT		= 0;
	localparam isTAG_RD		= 1;
	localparam isCACHE_RD	= 2;
	localparam isRESP		= 3;
	localparam isEVICT		= 4;
	localparam isMEM_RD		= 5;
	localparam isRECV		= 6;
	localparam isREFILL		= 7;

	wire ReadMiss;
	wire ReadHit;

	always @ (posedge clk) begin
		if (rst) begin
			current_state <= WAIT;
		end else begin
			current_state <= next_state;
		end
	end

	always @(*) begin
		case (current_state)
			WAIT: begin
				if (from_cpu_inst_req_valid) begin
					next_state <= TAG_RD;
				end else begin
					next_state <= WAIT;
				end
			end
			TAG_RD: begin
				if (ReadMiss) begin
					next_state <= EVICT;
				end else if (ReadHit) begin
					next_state <= CACHE_RD;
				end else begin
					next_state <= TAG_RD;
				end
			end
			CACHE_RD: next_state <= RESP;
			RESP: begin
				if (from_cpu_cache_rsp_ready) begin
					next_state <= WAIT;
				end else begin
					next_state <= RESP;
				end
			end
			EVICT: next_state <= MEM_RD;
			MEM_RD: begin
				if (from_mem_rd_req_ready) begin
					next_state <= RECV;
				end else begin
					next_state <= MEM_RD;
				end
			end
			RECV: begin
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
					next_state <= REFILL;
				end else begin 
					next_state <= RECV;
				end
			end
			REFILL: next_state <= RESP;
			default: next_state<= WAIT;
		endcase
	end

	assign {tag, index, offset} 	= from_cpu_inst_req_addr;
	assign to_cpu_inst_req_ready 	= current_state[isWAIT];
	assign to_cpu_cache_rsp_valid 	= current_state[isRESP];
	assign to_mem_rd_req_valid		= current_state[isMEM_RD];
	assign to_mem_rd_req_addr 		= {tag, index, 5'b00000};
	assign to_mem_rd_rsp_ready		= current_state[isRECV] || rst;

	genvar i_way;
	generate
		for (i_way=0;i_way<`CACHE_WAY;i_way=i_way+1) begin : ways
			tag_array tags(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen[i_way]),
				.wdata	(tag),
				.rdata	(tag_rdata[i_way])
			);
			assign tag_hit[i_way] = valid_array[i_way][index] && (tag_rdata[i_way] == tag);
			data_array datas(
				.clk	(clk),
				.waddr	(index),
				.raddr	(index),
				.wen	(wen[i_way]),
				.wdata	({
					data_recv[7],data_recv[6],data_recv[5],data_recv[4],
					data_recv[3],data_recv[2],data_recv[1],data_recv[0]
				}),
				.rdata	(data_rdata[i_way])
			);
		end
	endgenerate

	assign ReadHit	= tag_hit[0] | tag_hit[1] | tag_hit[2] | tag_hit[3];
	assign ReadMiss = ~ReadHit;
	
	assign way_selected = 	{(2){tag_hit[0]}} & 2'b00 |
							{(2){tag_hit[1]}} & 2'b01 |
							{(2){tag_hit[2]}} & 2'b10 |
							{(2){tag_hit[3]}} & 2'b11 ;
	assign data_selected = data_rdata[way_selected];
	/*	assign data_selected = 	{(`LINE_LEN){tag_hit[0]}} & data_rdata[0] |
	 *							{(`LINE_LEN){tag_hit[1]}} & data_rdata[1] |
	 *							{(`LINE_LEN){tag_hit[2]}} & data_rdata[2] |
	 *							{(`LINE_LEN){tag_hit[3]}} & data_rdata[3] ;
	 */
	//Wrong: A reference to a wire or reg
	//assign to_cpu_cache_rsp_data = data_selected[{offset[4:2],5'b11111}:{offset[4:2],5'b00000}];
	assign to_cpu_cache_rsp_data = data_selected >> {offset[4:0], 3'b000};

	integer i;
	always @(posedge clk) begin
		if (rst) begin
			for (i=0;i<8;i=i+1) begin
				LRU_cnt[0][i] <= 0;
				LRU_cnt[1][i] <= 0;
				LRU_cnt[2][i] <= 0;
				LRU_cnt[3][i] <= 0;
			end
		end else if (!rst && current_state[isRESP] && next_state[isWAIT]) begin
			if (way_selected!=2'b00) LRU_cnt[0][index] <= LRU_cnt[0][index] + 1;
			if (way_selected!=2'b01) LRU_cnt[1][index] <= LRU_cnt[1][index] + 1;
			if (way_selected!=2'b10) LRU_cnt[2][index] <= LRU_cnt[2][index] + 1;
			if (way_selected!=2'b11) LRU_cnt[3][index] <= LRU_cnt[3][index] + 1;
		end else if (!rst && current_state[isREFILL]) begin
			LRU_cnt[way_LRU[index]][index] <= 4'b0000;
		end
	end

	genvar i_set;
	generate
		for (i_set=0;i_set<`CACHE_SET;i_set=i_set+1) begin : sets
			assign way_LRU[i_set] = 
				{(2){~valid_array[0][i_set]}} & 2'b00 |
				{(2){ valid_array[0][i_set] & ~valid_array[1][i_set]}} & 2'b01 |
				{(2){ valid_array[0][i_set] &  valid_array[1][i_set] & ~valid_array[2][i_set]}} & 2'b10 |
				{(2){ valid_array[0][i_set] &  valid_array[1][i_set] &  valid_array[2][i_set] & ~valid_array[3][i_set]}} & 2'b11 |
				{(2){ valid_array[0][i_set] &  valid_array[1][i_set] &  valid_array[2][i_set] &  valid_array[3][i_set]}} & (
					LRU_cnt[0][i_set] >= LRU_cnt[1][i_set] ?
					LRU_cnt[0][i_set] >= LRU_cnt[2][i_set] ?
					LRU_cnt[0][i_set] >= LRU_cnt[3][i_set] ? 2'b00 : 2'b11 :
					LRU_cnt[2][i_set] >= LRU_cnt[3][i_set] ? 2'b10 : 2'b11 :
					LRU_cnt[1][i_set] >= LRU_cnt[2][i_set] ?
					LRU_cnt[1][i_set] >= LRU_cnt[3][i_set] ? 2'b01 : 2'b11 :
					LRU_cnt[2][i_set] >= LRU_cnt[3][i_set] ? 2'b10 : 2'b11
				);
		end
	endgenerate

	integer j;
	always @(posedge clk) begin
		if (rst) begin
			for (j=0;j<8;j=j+1) begin
				valid_array[0][j] <= 0;
				valid_array[1][j] <= 0;
				valid_array[2][j] <= 0;
				valid_array[3][j] <= 0;
			end
		end else if (!rst && current_state[isEVICT]) begin
			valid_array[way_LRU[index]][index]<=1'b0;
		end else if (!rst && current_state[isMEM_RD]) begin
			len <= 3'b000;
		end else if (!rst && current_state[isRECV] && from_mem_rd_rsp_valid) begin
			data_recv[len] <= from_mem_rd_rsp_data;
			len <= len + 1;
		end else if (!rst && current_state[isREFILL]) begin
			valid_array[way_LRU[index]][index] <= 1'b1;
		end
	end

	assign wen[0] = current_state[isREFILL] && way_LRU[index] == 2'b00;
	assign wen[1] = current_state[isREFILL] && way_LRU[index] == 2'b01;
	assign wen[2] = current_state[isREFILL] && way_LRU[index] == 2'b10;
	assign wen[3] = current_state[isREFILL] && way_LRU[index] == 2'b11;

endmodule

