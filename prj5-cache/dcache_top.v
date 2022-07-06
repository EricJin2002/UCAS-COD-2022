`timescale 10ns / 1ns
//`include "sources/custom_cpu/cache/tag_array.v"
//`include "sources/custom_cpu/cache/data_array.v"
`define CACHE_SET	8
`define CACHE_WAY	4
`define TAG_LEN		24
`define LINE_LEN	256

module dcache_top (
	input	      clk,
	input	      rst,
  
	//CPU interface
	/** CPU memory/IO access request to Cache: valid signal */
	input         from_cpu_mem_req_valid,
	/** CPU memory/IO access request to Cache: 0 for read; 1 for write (when req_valid is high) */
	input         from_cpu_mem_req,
	/** CPU memory/IO access request to Cache: address (4 byte alignment) */
	input  [31:0] from_cpu_mem_req_addr,
	/** CPU memory/IO access request to Cache: 32-bit write data */
	input  [31:0] from_cpu_mem_req_wdata,
	/** CPU memory/IO access request to Cache: 4-bit write strobe */
	input  [ 3:0] from_cpu_mem_req_wstrb,
	/** Acknowledgement from Cache: ready to receive CPU memory access request */
	output        to_cpu_mem_req_ready,
		
	/** Cache responses to CPU: valid signal */
	output        to_cpu_cache_rsp_valid,
	/** Cache responses to CPU: 32-bit read data */
	output [31:0] to_cpu_cache_rsp_data,
	/** Acknowledgement from CPU: Ready to receive read data */
	input         from_cpu_cache_rsp_ready,
		
	//Memory/IO read interface
	/** Cache sending memory/IO read request: valid signal */
	output        to_mem_rd_req_valid,
	/** Cache sending memory read request: address
	  * 4 byte alignment for I/O read 
	  * 32 byte alignment for cache read miss */
	output [31:0] to_mem_rd_req_addr,
        /** Cache sending memory read request: burst length
	  * 0 for I/O read (read only one data beat)
	  * 7 for cache read miss (read eight data beats) */
	output [ 7:0] to_mem_rd_req_len,
        /** Acknowledgement from memory: ready to receive memory read request */
	input	      from_mem_rd_req_ready,

	/** Memory return read data: valid signal of one data beat */
	input	      from_mem_rd_rsp_valid,
	/** Memory return read data: 32-bit one data beat */
	input  [31:0] from_mem_rd_rsp_data,
	/** Memory return read data: if current data beat is the last in this burst data transmission */
	input	      from_mem_rd_rsp_last,
	/** Acknowledgement from cache: ready to receive current data beat */
	output        to_mem_rd_rsp_ready,

	//Memory/IO write interface
	/** Cache sending memory/IO write request: valid signal */
	output        to_mem_wr_req_valid,
	/** Cache sending memory write request: address
	  * 4 byte alignment for I/O write 
	  * 4 byte alignment for cache write miss
          * 32 byte alignment for cache write-back */
	output [31:0] to_mem_wr_req_addr,
        /** Cache sending memory write request: burst length
          * 0 for I/O write (write only one data beat)
          * 0 for cache write miss (write only one data beat)
          * 7 for cache write-back (write eight data beats) */
	output [ 7:0] to_mem_wr_req_len,
        /** Acknowledgement from memory: ready to receive memory write request */
	input         from_mem_wr_req_ready,

	/** Cache sending memory/IO write data: valid signal for current data beat */
	output        to_mem_wr_data_valid,
	/** Cache sending memory/IO write data: current data beat */
	output [31:0] to_mem_wr_data,
	/** Cache sending memory/IO write data: write strobe
	  * 4'b1111 for cache write-back 
	  * other values for I/O write and cache write miss according to the original CPU request*/ 
	output [ 3:0] to_mem_wr_data_strb,
	/** Cache sending memory/IO write data: if current data beat is the last in this burst data transmission */
	output        to_mem_wr_data_last,
	/** Acknowledgement from memory/IO: ready to receive current data beat */
	input	      from_mem_wr_data_ready
);

  //TODO: Please add your D-Cache code here
	
	reg						  	valid_array	[`CACHE_WAY - 1:0][`CACHE_SET - 1:0];
	reg						  	dirty_array	[`CACHE_WAY - 1:0][`CACHE_SET - 1:0];
	
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

	reg  [			   31:0]	data_wb;
	reg  [			   31:0]	data_bypass;

	reg  [31:0] Address;
	reg  [31:0] Write_data;
	reg  [3:0]	Write_strb;
	reg			Mem_req;

	wire [23:0] tag;
	wire [ 2:0] index;
	wire [ 4:0] offset;

	wire Cacheable;
	wire Miss;
	wire Dirty;

	reg [15:0] current_state;
	reg [15:0] next_state;

	localparam WAIT			= 16'b0000_0000_0000_0001;
	localparam TAG_RD		= 16'b0000_0000_0000_0010;
	localparam EVICT		= 16'b0000_0000_0000_0100;
	localparam MEM_WR		= 16'b0000_0000_0000_1000;
	localparam CACHE_WB		= 16'b0000_0000_0001_0000;
	localparam MEM_RD		= 16'b0000_0000_0010_0000;
	localparam RECV_MEM		= 16'b0000_0000_0100_0000;
	localparam REFILL		= 16'b0000_0000_1000_0000;
	localparam CACHE_WR		= 16'b0000_0001_0000_0000;
	localparam CACHE_RD		= 16'b0000_0010_0000_0000;
	localparam CACHE_RESP	= 16'b0000_0100_0000_0000;	
	localparam BYPASS_RD	= 16'b0000_1000_0000_0000;	
	localparam MEM_RESP		= 16'b0001_0000_0000_0000;
	localparam CPU_RECV		= 16'b0010_0000_0000_0000;
	localparam BYPASS_WR	= 16'b0100_0000_0000_0000;	
	localparam CPU_WB		= 16'b1000_0000_0000_0000;

	localparam isWAIT		= 0;
	localparam isTAG_RD		= 1;
	localparam isEVICT		= 2;
	localparam isMEM_WR		= 3;
	localparam isCACHE_WB	= 4;	
	localparam isMEM_RD		= 5;
	localparam isRECV_MEM	= 6;	
	localparam isREFILL		= 7;
	localparam isCACHE_WR	= 8;	
	localparam isCACHE_RD	= 9;	
	localparam isCACHE_RESP	= 10;	
	localparam isBYPASS_RD	= 11;	
	localparam isMEM_RESP	= 12;	
	localparam isCPU_RECV	= 13;
	localparam isBYPASS_WR	= 14;	
	localparam isCPU_WB		= 15;

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
				if (from_cpu_mem_req_valid && Cacheable) begin
					next_state <= TAG_RD;
				end else if (from_cpu_mem_req_valid && !Cacheable && from_cpu_mem_req) begin
					next_state <= BYPASS_WR;
				end else if (from_cpu_mem_req_valid && !Cacheable && !from_cpu_mem_req) begin
					next_state <= BYPASS_RD;
				end else begin
					next_state <= WAIT;
				end
			end
			TAG_RD: begin
				if (Miss) begin
					next_state <= EVICT;
				end else if (!Miss && Mem_req) begin
					next_state <= CACHE_WR;
				end else if (!Miss && !Mem_req) begin
					next_state <= CACHE_RD;
				end else begin
					next_state <= TAG_RD;
				end
			end
			EVICT: begin
				if (Dirty) begin
					next_state <= MEM_WR;
				end else if (!Dirty) begin
					next_state <= MEM_RD;
				end else begin
					next_state <= EVICT;
				end
			end	 
			MEM_WR: begin
				if (from_mem_wr_req_ready) begin
					next_state <= CACHE_WB;
				end else begin
					next_state <= MEM_WR;
				end
			end
			CACHE_WB: begin
				if (from_mem_wr_data_ready && to_mem_wr_data_last) begin
					next_state <= MEM_RD;
				end else begin
					next_state <= CACHE_WB;
				end
			end
			MEM_RD: begin
				if (from_mem_rd_req_ready) begin
					next_state <= RECV_MEM;
				end else begin
					next_state <= MEM_RD;
				end
			end		 
			RECV_MEM: begin
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
					next_state <= REFILL;
				end else begin
					next_state <= RECV_MEM;
				end
			end 
			REFILL: begin
				if (Mem_req) begin
					next_state <= CACHE_WR;
				end else if (!Mem_req) begin
					next_state <= CACHE_RD;
				end else begin
					next_state <= REFILL;
				end
			end
			CACHE_WR: begin
				next_state <= WAIT;
			end
			CACHE_RD: begin
				next_state <= CACHE_RESP;
			end
			CACHE_RESP: begin
				if (from_cpu_cache_rsp_ready) begin
					next_state <= WAIT;
				end else begin
					next_state <= CACHE_RESP;
				end
			end
			BYPASS_RD: begin
				if (from_mem_rd_req_ready) begin
					next_state <= MEM_RESP;
				end else begin
					next_state <= BYPASS_RD;
				end
			end
			MEM_RESP: begin
				if (from_mem_rd_rsp_valid && from_mem_rd_rsp_last) begin
					next_state <= CPU_RECV;
				end else begin
					next_state <= MEM_RESP;
				end
			end
			CPU_RECV: begin
				if (from_cpu_cache_rsp_ready) begin
					next_state <= WAIT;
				end else begin
					next_state <= CPU_RECV;
				end
			end
			BYPASS_WR: begin
				if (from_mem_wr_req_ready) begin
					next_state <= CPU_WB;
				end else begin
					next_state <= BYPASS_WR;
				end
			end
			CPU_WB: begin
				if (from_mem_wr_data_ready && to_mem_wr_data_last) begin
					next_state <= WAIT;
				end else begin
					next_state <= CPU_WB;
				end
			end
			default: next_state <= WAIT;
		endcase
	end

	assign to_cpu_mem_req_ready		= current_state[isWAIT];
	assign to_cpu_cache_rsp_valid	= current_state[isCACHE_RESP] || current_state[isCPU_RECV];
	assign to_cpu_cache_rsp_data	= {(32){current_state[isCACHE_RESP]}}	& data_selected >> {offset[4:0], 3'b000}
									| {(32){current_state[isCPU_RECV]}}		& data_bypass;
	assign to_mem_rd_req_valid		= current_state[isMEM_RD] || current_state[isBYPASS_RD];
	assign to_mem_rd_req_addr		= {(32){current_state[isMEM_RD]}} 	 & {tag, index, 5'b00000}
									| {(32){current_state[isBYPASS_RD]}} & Address;
	assign to_mem_rd_req_len		= {( 8){current_state[isMEM_RD]}} 	 & 7
									| {( 8){current_state[isBYPASS_RD]}} & 0;
	assign to_mem_rd_rsp_ready		= current_state[isRECV_MEM] || current_state[isMEM_RESP] || rst;
	assign to_mem_wr_req_valid		= current_state[isMEM_WR] || current_state[isBYPASS_WR];
	assign to_mem_wr_req_addr		= {(32){current_state[isMEM_WR]}}	 & {tag_rdata[way_LRU[index]], index, 5'b00000}
									| {(32){current_state[isBYPASS_WR]}} & Address;
	assign to_mem_wr_req_len		= {( 8){current_state[isMEM_WR]}} 	 & 7
									| {( 8){current_state[isBYPASS_WR]}} & 0;
	assign to_mem_wr_data_valid		= current_state[isCACHE_WB] || current_state[isCPU_WB];
	assign to_mem_wr_data			= {(32){current_state[isCACHE_WB]}}	& data_wb
									| {(32){current_state[isCPU_WB]}}	& Write_data;
	assign to_mem_wr_data_strb		= {( 4){current_state[isCACHE_WB]}}
									| {( 4){current_state[isCPU_WB]}}	& Write_strb;
	assign to_mem_wr_data_last		= current_state[isCACHE_WB] && len == 3'b000
								   || current_state[isCPU_WB];

	assign {tag, index, offset} = Address;

	assign Cacheable 	= (|from_cpu_mem_req_addr[31:5]) && !from_cpu_mem_req_addr[31] && !from_cpu_mem_req_addr[30];
	assign Miss 		= !(tag_hit[0] || tag_hit[1] || tag_hit[2] || tag_hit[3]);
	assign Dirty 		= valid_array[way_LRU[index]][index] && dirty_array[way_LRU[index]][index];
	
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
				.wdata	({(256){current_state[isREFILL]}} & {
					data_recv[7],data_recv[6],data_recv[5],data_recv[4],
					data_recv[3],data_recv[2],data_recv[1],data_recv[0]
				} | {(256){current_state[isCACHE_WR]}} & (
					data_rdata[i_way] & ~({
						224'b0,
						{(8){Write_strb[3]}},{(8){Write_strb[2]}},{(8){Write_strb[1]}},{(8){Write_strb[0]}}
					}<<{offset,3'b000}) | ({
						224'b0,
						Write_data & {{(8){Write_strb[3]}},{(8){Write_strb[2]}},{(8){Write_strb[1]}},{(8){Write_strb[0]}}}
					}<<{offset,3'b000})
				)),
				.rdata	(data_rdata[i_way])
			);
		end
	endgenerate

	assign wen[0] = current_state[isREFILL] && way_LRU[index] == 2'b00 ||
					current_state[isCACHE_WR] && way_selected == 2'b00;
	assign wen[1] = current_state[isREFILL] && way_LRU[index] == 2'b01 ||
					current_state[isCACHE_WR] && way_selected == 2'b01;
	assign wen[2] = current_state[isREFILL] && way_LRU[index] == 2'b10 ||
					current_state[isCACHE_WR] && way_selected == 2'b10;
	assign wen[3] = current_state[isREFILL] && way_LRU[index] == 2'b11 ||
					current_state[isCACHE_WR] && way_selected == 2'b11;

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
	
	assign way_selected	 = 	{(2){tag_hit[0]}} & 2'b00 |
							{(2){tag_hit[1]}} & 2'b01 |
							{(2){tag_hit[2]}} & 2'b10 |
							{(2){tag_hit[3]}} & 2'b11 ;
	assign data_selected = data_rdata[way_selected];

	always @(posedge clk) begin
		if (current_state[isWAIT]) begin
			Address		<= from_cpu_mem_req_addr;
			Write_data	<= from_cpu_mem_req_wdata;
			Write_strb	<= from_cpu_mem_req_wstrb;
			Mem_req		<= from_cpu_mem_req;
		end
	end

	integer j;
	always @(posedge clk) begin
		if (rst) begin
			for (j=0;j<8;j=j+1) begin
				valid_array[0][j] <= 0;
				valid_array[1][j] <= 0;
				valid_array[2][j] <= 0;
				valid_array[3][j] <= 0;
				LRU_cnt[0][j] <= 0;
				LRU_cnt[1][j] <= 0;
				LRU_cnt[2][j] <= 0;
				LRU_cnt[3][j] <= 0;
			end
		end else if (!rst && current_state[isEVICT]) begin
			valid_array[way_LRU[index]][index] <= 1'b0; //keep zero till finish clean
		//end else if (next_state[isMEM_WR]) begin //for mutually excluded if
			len <= 3'b000; //initialize the counter `len`
		end else if (!rst && !current_state[isCACHE_WB] && next_state[isCACHE_WB]) begin
			data_wb <= data_rdata[way_LRU[index]] >> {len, 5'b00000};
			len <= len + 1;
		end else if (!rst && current_state[isCACHE_WB] && from_mem_wr_data_ready) begin
			data_wb <= data_rdata[way_LRU[index]] >> {len, 5'b00000};
			len <= len + 1;
		end else if (!rst && current_state[isMEM_RD]) begin
			len <= 3'b000;
		end else if (!rst && current_state[isRECV_MEM] && from_mem_rd_rsp_valid) begin
			data_recv[len] <= from_mem_rd_rsp_data;
			len <= len + 1;
		end else if (!rst && current_state[isREFILL]) begin
			LRU_cnt[way_LRU[index]][index] <= 4'b0000;
			valid_array[way_LRU[index]][index] <= 1'b1; //finish clean
			dirty_array[way_LRU[index]][index] <= 1'b0;
		end else if (!rst && current_state[isCACHE_RESP] && next_state[isWAIT]) begin
			if (way_selected!=2'b00) LRU_cnt[0][index] <= LRU_cnt[0][index] + 1;
			if (way_selected!=2'b01) LRU_cnt[1][index] <= LRU_cnt[1][index] + 1;
			if (way_selected!=2'b10) LRU_cnt[2][index] <= LRU_cnt[2][index] + 1;
			if (way_selected!=2'b11) LRU_cnt[3][index] <= LRU_cnt[3][index] + 1;
		end else if (!rst && current_state[isCACHE_WR]) begin
			dirty_array[way_selected][index] <= 1'b1;
		end else if (!rst && current_state[isMEM_RESP] && from_mem_rd_rsp_valid) begin
			data_bypass <= from_mem_rd_rsp_data;
		end
	end

endmodule

