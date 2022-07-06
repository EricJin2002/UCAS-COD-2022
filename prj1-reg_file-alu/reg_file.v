`timescale 10 ns / 1 ns

`define DATA_WIDTH 32
`define ADDR_WIDTH 5

module reg_file(
	input                       clk,
	input  [`ADDR_WIDTH - 1:0]  waddr,
	input  [`ADDR_WIDTH - 1:0]  raddr1,
	input  [`ADDR_WIDTH - 1:0]  raddr2,
	input                       wen,
	input  [`DATA_WIDTH - 1:0]  wdata,
	output [`DATA_WIDTH - 1:0]  rdata1,
	output [`DATA_WIDTH - 1:0]  rdata2
);

	// TODO: Please add your logic design here
	reg [`DATA_WIDTH - 1:0] r [(1<<`ADDR_WIDTH) - 1:0];
	always @(posedge clk) begin
		if(wen==1'b1 && waddr) r[waddr] <= wdata;
	end
	assign rdata1 = raddr1 ? r[raddr1] : `DATA_WIDTH'b0;
	assign rdata2 = raddr2 ? r[raddr2] : `DATA_WIDTH'b0;
endmodule
