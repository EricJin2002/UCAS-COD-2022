`timescale 10 ns / 1 ns

`define DATA_WIDTH 32

module alu(
	input  [`DATA_WIDTH - 1:0]  A,
	input  [`DATA_WIDTH - 1:0]  B,
	input  [              2:0]  ALUop,
	output                      Overflow,
	output                      CarryOut,
	output                      Zero,
	output [`DATA_WIDTH - 1:0]  Result
);
	// TODO: Please add your logic design here
	wire [`DATA_WIDTH - 1:0] AddResult;
	wire [`DATA_WIDTH:0] Bnew;
	wire [`DATA_WIDTH - 1:0] sig [7:0];
	wire minus;

	// Decode ALUop[2:0] to sig[7:0]
	assign sig[3'b000] = {(`DATA_WIDTH){!ALUop[2] & !ALUop[1] & !ALUop[0]}};
	assign sig[3'b001] = {(`DATA_WIDTH){!ALUop[2] & !ALUop[1] &  ALUop[0]}};
	assign sig[3'b010] = {(`DATA_WIDTH){!ALUop[2] &  ALUop[1] & !ALUop[0]}};
	assign sig[3'b011] = {(`DATA_WIDTH){!ALUop[2] &  ALUop[1] &  ALUop[0]}};
	assign sig[3'b100] = {(`DATA_WIDTH){ ALUop[2] & !ALUop[1] & !ALUop[0]}};
	assign sig[3'b101] = {(`DATA_WIDTH){ ALUop[2] & !ALUop[1] &  ALUop[0]}};
	assign sig[3'b110] = {(`DATA_WIDTH){ ALUop[2] &  ALUop[1] & !ALUop[0]}};
	assign sig[3'b111] = {(`DATA_WIDTH){ ALUop[2] &  ALUop[1] &  ALUop[0]}};

	// Generate results, carryout, overflow
	// Judge to do minus or not
	assign minus = ALUop[2] | ALUop[0];
	// If slt or sub or sltu, reverse B
	assign Bnew = {(`DATA_WIDTH + 1){minus}} ^ {1'b0, B};
	// Use one adder to generate result and carryout for add and sub
	assign {CarryOut, AddResult} = {1'b0, A} + Bnew + minus;
	/* Set overflow to 1
	 * if pos + pos = neg
	 * or pos - neg = neg
	 * or neg + neg = pos
	 * or neg - pos = pos
	 */ 
	assign Overflow = A[`DATA_WIDTH-1] & (ALUop[2]^B[`DATA_WIDTH-1]) & ~AddResult[`DATA_WIDTH-1]
		| ~A[`DATA_WIDTH-1] & (ALUop[2]^~B[`DATA_WIDTH-1]) & AddResult[`DATA_WIDTH-1];

	// Select result according to sig[3:0]
	assign Result = sig[3'b000] & (A&B)
		| sig[3'b001] & (A|B)
		| sig[3'b100] & (A^B)
		| sig[3'b101] & ~(A|B)
		| sig[3'b010] & AddResult
		| sig[3'b110] & AddResult
		| sig[3'b111] & (Overflow^AddResult[`DATA_WIDTH-1])
		| sig[3'b011] & CarryOut;

	// Generate zero
	assign Zero = ~|Result;

endmodule
