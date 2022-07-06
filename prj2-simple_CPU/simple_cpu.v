`timescale 10ns / 1ns
//`include "sources/reg_file/reg_file.v"
//`include "sources/alu/alu.v"
//`include "sources/shifter/shifter.v"
module simple_cpu(
	input             clk,
	input             rst,

	output [31:0]     PC,
	input  [31:0]     Instruction,

	output [31:0]     Address,
	output            MemWrite,
	output [31:0]     Write_data,
	output [ 3:0]     Write_strb,

	input  [31:0]     Read_data,
	output            MemRead
);

	// THESE THREE SIGNALS ARE USED IN OUR TESTBENCH
	// PLEASE DO NOT MODIFY SIGNAL NAMES
	// AND PLEASE USE THEM TO CONNECT PORTS
	// OF YOUR INSTANTIATION OF THE REGISTER FILE MODULE
	wire			RF_wen;
	wire [4:0]		RF_waddr;
	wire [31:0]		RF_wdata;

	// TODO: PLEASE ADD YOUR CODE BELOW

	wire R_Type;
	wire REGIMM;
	wire J_Type;
	wire I_Type_b;
	wire I_Type_calc;
	wire I_Type_mr;
	wire I_Type_mw;

	//FSM
	reg [4:0] current_state;
	reg [4:0] next_state;

	localparam IFSTATE	= 5'b00001;
	localparam IDSTATE	= 5'b00010;
	localparam EXSTATE	= 5'b00100;
	localparam MEMSTATE	= 5'b01000;
	localparam WBSTATE	= 5'b10000;

	always @ (posedge clk) begin
		if(rst) begin
			current_state <= IFSTATE;
		end else begin
			current_state <= next_state;
		end
	end

	always @(*) begin
		case (current_state)
			IFSTATE: next_state <= IDSTATE;
			IDSTATE: begin
				if(|Instruction_cache) begin
					next_state <= EXSTATE;
				end else begin
					next_state <= IFSTATE;
				end
			end
			EXSTATE: begin
				if(R_Type | I_Type_calc | J_Type & opcode[0]) begin
					next_state <= WBSTATE;
				end else if(I_Type_mr | I_Type_mw) begin
					next_state <= MEMSTATE;
				end else begin
					next_state <= IFSTATE;
				end
			end
			MEMSTATE: begin
				if(I_Type_mr) begin
					next_state <= WBSTATE;
				end else begin
					next_state <= IFSTATE;
				end 
			end
			WBSTATE: next_state <= IFSTATE;
			default: next_state <= IDSTATE;
		endcase
	end


	//IF

	reg [31:0] Instruction_cache;
	wire [5:0] opcode;
	wire [4:0] rs;
	wire [4:0] rt;
	wire [4:0] rd;
	wire [4:0] shamt;
	wire [5:0] func;
	assign {opcode,rs,rt,rd,shamt,func} = Instruction_cache;
	always @(posedge clk) begin
		if (current_state == IFSTATE) begin
			Instruction_cache <= Instruction;
		end
	end

	//Update PC
	reg [31:0] PC;
	always @(posedge clk) begin
		if (rst) begin
			PC <= 32'd0;
		end else if (current_state == IFSTATE) begin
			PC <= PC + 4;
		end else if (current_state == EXSTATE) begin
			if (R_Type_jump) begin
				PC <= RF_rdata1;
			end else if (J_Type) begin
				PC <= {Instruction_cache[25:0], 2'b00};	
			end else if ((Zero ^ (
				REGIMM & ~Instruction_cache[16] 
				| I_Type_b & (opcode[0] ^ (opcode[1] & |RF_rdata1))
			)) & Branch) begin
				PC <= PC + {SignExtend[29:0], 2'b00};
			end
		end
	end


	//ID

	wire RegDst,Branch,MemtoReg,ALUEn,ShiftEn,ALUSrc,ShiftSrc;

	//ALU Control
	assign R_Type		= opcode[5:0] == 6'b000000;
	assign R_Type_calc	= R_Type & func[5];
	assign R_Type_shift	= R_Type & (func[5:3]==3'b000);
	assign R_Type_jump	= R_Type & ({func[5:3],func[1]}==4'b0010);
	assign R_Type_mov	= R_Type & ({func[5:3],func[1]}==4'b0011);
	assign REGIMM 		= opcode[5:0] == 6'b000001;
	assign J_Type 		= opcode[5:1] == 5'b00001;
	assign I_Type_b 	= opcode[5:2] == 4'b0001;
	assign I_Type_calc 	= opcode[5:3] == 3'b001;
	assign I_Type_mr	= opcode[5:3] == 3'b100;
	assign I_Type_mw	= opcode[5:3] == 3'b101;

	assign RegDst 	= R_Type;
	assign Branch 	= REGIMM | I_Type_b;
	assign MemRead	= (current_state==MEMSTATE) & I_Type_mr;
	assign MemtoReg	= I_Type_mr;
	assign ALUEn	= R_Type_calc | REGIMM | I_Type_b | I_Type_calc | I_Type_mr | I_Type_mw;
	assign ShiftEn	= R_Type_shift;
	assign MemWrite = (current_state==MEMSTATE) & I_Type_mw;
	assign ALUSrc	= I_Type_calc | I_Type_mr | I_Type_mw;
	assign ShiftSrc = func[2];
	assign RF_wen	= (current_state==WBSTATE) & (
						R_Type	& (~R_Type_mov | (func[0]^~|RF_rdata2))
								& (~R_Type_jump | func[0]) // jalr allowed, jr not
								 /* must add the line above 
							  	  * since the benchmark judges RF_wen==1 (when jr) to be wrong
							  	  * even if RF_waddr==0
							  	  * when the RF naturally prevent data from writing in
							  	  */
						| J_Type & opcode[0]
						| I_Type_calc
						| I_Type_mr
					);

	//Registers
	wire [31:0] RF_rdata1_wire;
	wire [31:0] RF_rdata2_wire;
	reg  [31:0] RF_rdata1;
	reg  [31:0] RF_rdata2;
	reg_file Registers(
		.clk	(clk),
		.waddr	(RF_waddr),
		.raddr1	(rs),
		.raddr2	(rt),
		.wen	(RF_wen),
		.wdata	(RF_wdata),
		.rdata1	(RF_rdata1_wire),
		.rdata2	(RF_rdata2_wire)
	);
	assign RF_waddr = RegDst ? rd : {(5){~REGIMM}} & ({(5){J_Type}} | rt);
	always @(posedge clk) begin
		if (current_state == IDSTATE) begin
			RF_rdata1 <= RF_rdata1_wire;
			RF_rdata2 <= RF_rdata2_wire;
		end
	end

	//Sign extend
	reg [31:0] SignExtend;
	//Zero extend for I-Type andi, ori, xori
	reg [31:0] ZeroExtend;
	//Select
	wire [31:0] ExtendedImm;
	assign ExtendedImm = opcode[5:2] == 4'b0011 ? ZeroExtend : SignExtend;
	always @(posedge clk) begin
		if (current_state == IDSTATE) begin
			SignExtend <= {{(16){Instruction_cache[15]}}, Instruction_cache[15:0]};
			ZeroExtend <= {16'b0, Instruction_cache[15:0]};
		end
	end

	//for R_Type_jump | J_Type
	reg [31:0] PC_normal;
	always @(posedge clk) begin
		if (current_state == IDSTATE) begin
			PC_normal <= PC;
		end
	end

	
	//EX

	wire [2:0] ALUop;
	wire Overflow,CarryOut,Zero;
	wire [31:0] ALUResult,ShifterResult;
	reg  [31:0] Result;
	wire [1:0] Shiftop;

	//ALU
	alu ALU(
		.A			(RF_rdata1),
		.B			(ALUSrc ? ExtendedImm : REGIMM ? 32'b0 :RF_rdata2),
		.ALUop		(ALUop),
		.Overflow	(Overflow),
		.CarryOut	(CarryOut),
		.Zero		(Zero),
		.Result		(ALUResult)
	);
	assign ALUop = {(3){R_Type_calc | R_Type_jump}} & {
		func[1] & ~(func[3] & func[0]),
		~func[2],
		func[3] & ~func[2] & func[1] | func[2] & func[0]
	} | {(3){I_Type_calc}} & {
		opcode[1] & ~(opcode[3] & opcode[0]),
		~opcode[2],
		opcode[3] & ~opcode[2] & opcode[1] | opcode[2] & opcode[0]
	} | {(3){REGIMM}}
	  | {(3){I_Type_b}}	 & {2'b11, opcode[1]}	// slt 111 sub 110
	  | {(3){I_Type_mr | I_Type_mw}} & 3'b010;
	
	//Shifter
	shifter Shifter(
		.A			(RF_rdata2),
		.B			(ShiftSrc ? RF_rdata1[4:0] : shamt),
		.Shiftop	(Shiftop),
		.Result		(ShifterResult)
	);
	assign Shiftop = func[1:0];

	//Result
	always @(posedge clk) begin
		if (current_state == EXSTATE) begin
			Result <= {(32){ALUEn}} & ALUResult
					| {(32){ShiftEn}} & ShifterResult;
		end
	end


	//MEM

	//Data Memory
	//Write
	assign Address 		= Result & ~32'b11;
	assign Write_data	= opcode[2:0] == 3'b010 ? RF_rdata2 >> {~Result[1:0], 3'b0} : RF_rdata2 << {Result[1:0], 3'b0};
	assign Write_strb	= 	{(4){~opcode[2] & opcode[1] & ~opcode[0]}} & {	// swl
 								Result[1] & Result[0], Result[1], Result[1] | Result[0], 1'b1
						  	}
						|	{(4){ opcode[2] & opcode[1] & ~opcode[0]}} & {	// swr
 								1'b1, ~(Result[1]&Result[0]), ~Result[1], ~(Result[1]|Result[0])
						  	}
						|  	{(4){~opcode[1] | opcode[0]}} & {
								( Result[1] | opcode[1]) & ( Result[0] | opcode[0]),
						   	 	( Result[1] | opcode[1]) & (~Result[0] | opcode[0]),
						   	 	(~Result[1] | opcode[1]) & ( Result[0] | opcode[0]),
						     	(~Result[1] | opcode[1]) & (~Result[0] | opcode[0]) 
							};
	//Read
	reg [31:0] Read_data_reg;
	always @(posedge clk) begin
		if (current_state == MEMSTATE) begin
			Read_data_reg <= Read_data;
		end
	end
	

	//WB

	wire [31:0] Read_data_shifted;
	wire [31:0] Read_data_masked;
	wire Read_data_sign_bit;
	wire [31:0] Read_data_unaligned;
	assign Read_data_shifted 	= Read_data_reg >> {Result[1:0], 3'b0};
	assign Read_data_masked 	= Read_data_shifted & {{(16){opcode[1]}}, {(8){opcode[0]}}, {(8){1'b1}}}
								| {(32){~opcode[2] & Read_data_sign_bit}} & ~{{(16){opcode[1]}}, {(8){opcode[0]}}, {(8){1'b1}}};
	assign Read_data_sign_bit 	= Read_data_shifted[opcode[1:0]==2'b01 ? 15 : 7];
	assign Read_data_unaligned 	= {(32){~opcode[2]}} & (
									(Read_data_reg << {~Result[1:0], 3'b0}) | RF_rdata2 & ({(32){1'b1}} >> { Result[1:0], 3'b0})
								)|{(32){opcode[2]}} & (
									(Read_data_reg >> { Result[1:0], 3'b0}) | RF_rdata2 & ({(32){1'b1}} << {~Result[1:0], 3'b0})
								);
	assign RF_wdata = {(32){MemtoReg & (~opcode[1] |  opcode[0])}} & Read_data_masked
					| {(32){MemtoReg & ( opcode[1] & ~opcode[0])}} & Read_data_unaligned
					| {(32){R_Type_mov}} & RF_rdata1 
					| {(32){R_Type_jump | J_Type}} & PC_normal + 4
					| {(32){I_Type_calc & (&opcode[3:0])}} & {Instruction_cache[15:0], 16'd0}
					| {(32){R_Type_calc | R_Type_shift | I_Type_calc & ~&opcode[3:0]}} & Result;


endmodule
