`timescale 10ns / 1ns
//`include "sources/reg_file/reg_file.v"
//`include "sources/alu/alu.v"
//`include "sources/shifter/shifter.v"
module custom_cpu(
	input         clk,
	input         rst,

	//Instruction request channel
	output [31:0] PC,
	output        Inst_Req_Valid,
	input         Inst_Req_Ready,

	//Instruction response channel
	input  [31:0] Instruction,
	input         Inst_Valid,
	output        Inst_Ready,

	//Memory request channel
	output [31:0] Address,
	output        MemWrite,
	output [31:0] Write_data,
	output [ 3:0] Write_strb,
	output        MemRead,
	input         Mem_Req_Ready,

	//Memory data response channel
	input  [31:0] Read_data,
	input         Read_data_Valid,
	output        Read_data_Ready,

	input         intr,

	output [31:0] cpu_perf_cnt_0,
	output [31:0] cpu_perf_cnt_1,
	output [31:0] cpu_perf_cnt_2,
	output [31:0] cpu_perf_cnt_3,
	output [31:0] cpu_perf_cnt_4,
	output [31:0] cpu_perf_cnt_5,
	output [31:0] cpu_perf_cnt_6,
	output [31:0] cpu_perf_cnt_7,
	output [31:0] cpu_perf_cnt_8,
	output [31:0] cpu_perf_cnt_9,
	output [31:0] cpu_perf_cnt_10,
	output [31:0] cpu_perf_cnt_11,
	output [31:0] cpu_perf_cnt_12,
	output [31:0] cpu_perf_cnt_13,
	output [31:0] cpu_perf_cnt_14,
	output [31:0] cpu_perf_cnt_15,

	output [69:0] inst_retire
);

/* The following signal is leveraged for behavioral simulation, 
* which is delivered to testbench.
*
* STUDENTS MUST CONTROL LOGICAL BEHAVIORS of THIS SIGNAL.
*
* inst_retired (70-bit): detailed information of the retired instruction,
* mainly including (in order) 
* { 
*   reg_file write-back enable  (69:69,  1-bit),
*   reg_file write-back address (68:64,  5-bit), 
*   reg_file write-back data    (63:32, 32-bit),  
*   retired PC                  (31: 0, 32-bit)
* }
*
*/
  wire [69:0] inst_retire;

// TODO: Please add your custom CPU code here

	reg [8:0] current_state;
	reg [8:0] next_state;

	localparam INIT	=9'b000000001;
	localparam IF	=9'b000000010;
	localparam IW	=9'b000000100;
	localparam ID	=9'b000001000;
	localparam EX	=9'b000010000;
	localparam ST	=9'b000100000;
	localparam LD	=9'b001000000;
	localparam RDW	=9'b010000000;
	localparam WB	=9'b100000000;

	localparam isINIT	=0;
	localparam isIF 	=1;
	localparam isIW 	=2;
	localparam isID 	=3;
	localparam isEX 	=4;
	localparam isST 	=5;
	localparam isLD 	=6;
	localparam isRDW 	=7;
	localparam isWB 	=8;

	reg  [31:0]	PC;
	wire [31:0]	PC_branch;
	wire		Branch_or_not;
	reg  [31:0]	PC_normal;

	reg  [31:0]	IR;
	wire [5:0] 	opcode;
	wire [4:0] 	rs;
	wire [4:0] 	rt;
	wire [4:0] 	rd;
	wire [4:0] 	shamt;
	wire [5:0] 	func;

	wire R_Type;
	wire R_Type_calc;
	wire R_Type_shift;
	wire R_Type_jump;
	wire R_Type_mov;
	wire REGIMM;
	wire J_Type;
	wire I_Type_b;
	wire I_Type_calc;
	wire I_Type_mr;
	wire I_Type_mw;

	wire RegDst,Branch,MemtoReg,ALUEn,ShiftEn,ALUSrc,ShiftSrc,RF_wen;

	wire [31:0] RF_rdata1_wire;
	wire [31:0] RF_rdata2_wire;
	reg  [31:0] RF_rdata1;
	reg  [31:0] RF_rdata2;
	wire [4:0]	RF_waddr;
	wire [31:0]	RF_wdata;

	reg  [31:0] SignExtend; //Sign extend
	reg  [31:0] ZeroExtend; //Zero extend for I-Type andi, ori, xori
	wire [31:0] ExtendedImm; //Select

	wire [2:0]	ALUop;
	wire [1:0]	Shiftop;
	wire 		Overflow,CarryOut,Zero;
	wire [31:0]	ALUResult,ShifterResult;
	reg  [31:0]	Result;

	reg  [31:0]	MDR;

	wire [31:0]	Read_data_shifted;
	wire [31:0]	Read_data_masked;
	wire 		Read_data_sign_bit;
	wire [31:0]	Read_data_unaligned;


	assign inst_retire [69:69] = RF_wen;
	assign inst_retire [68:64] = RF_waddr;
	assign inst_retire [63:32] = RF_wdata;
	assign inst_retire [31: 0] = PC;


	//cnt
	reg  [31:0] cycle_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			cycle_cnt <= 32'd0;
		end else begin
			cycle_cnt <= cycle_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_0 = cycle_cnt;

	reg  [31:0] inst_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			inst_cnt <= 32'd0;
		end else if (current_state[isID]) begin
			inst_cnt <= inst_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_1 = inst_cnt;

	reg  [31:0] mr_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			mr_cnt <= 32'd0;
		end else if (current_state[isEX] && I_Type_mr) begin
			mr_cnt <= mr_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_2 = mr_cnt;

	reg  [31:0] mw_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			mw_cnt <= 32'd0;
		end else if (current_state[isEX] && I_Type_mw) begin
			mw_cnt <= mw_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_3 = mw_cnt;

	reg  [31:0] inst_req_delay_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			inst_req_delay_cnt <= 32'd0;
		end else if (current_state[isIF] && next_state[isIF]) begin
			inst_req_delay_cnt <= inst_req_delay_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_4 = inst_req_delay_cnt;

	reg  [31:0] inst_delay_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			inst_delay_cnt <= 32'd0;
		end else if (current_state[isIW] && next_state[isIW]) begin
			inst_delay_cnt <= inst_delay_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_5 = inst_delay_cnt;

	reg  [31:0] mr_req_delay_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			mr_req_delay_cnt <= 32'd0;
		end else if (current_state[isLD] && next_state[isLD]) begin
			mr_req_delay_cnt <= mr_req_delay_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_6 = mr_req_delay_cnt;

	reg  [31:0] rd_delay_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			rd_delay_cnt <= 32'd0;
		end else if (current_state[isRDW] && next_state[isRDW]) begin
			rd_delay_cnt <= rd_delay_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_7 = rd_delay_cnt;

	reg  [31:0] mw_req_delay_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			mw_req_delay_cnt <= 32'd0;
		end else if (current_state[isST] && next_state[isST]) begin
			mw_req_delay_cnt <= mw_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_8 = mw_req_delay_cnt;

	reg  [31:0] branch_inst_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			branch_inst_cnt <= 32'd0;
		end else if (current_state[isID] && (I_Type_b || REGIMM)) begin
			branch_inst_cnt <= branch_inst_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_9 = branch_inst_cnt;

	reg  [31:0] jump_inst_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			jump_inst_cnt <= 32'd0;
		end else if (current_state[isID] && (R_Type_jump || J_Type)) begin
			jump_inst_cnt <= jump_inst_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_10 = jump_inst_cnt;
	

	//FSM
	always @ (posedge clk) begin
		if (rst) begin
			current_state <= INIT;
		end else begin
			current_state <= next_state;
		end
	end

	always @(*) begin
		case (current_state)
			INIT: next_state <= IF;
			IF: begin
				if (Inst_Req_Ready) begin
					next_state <= IW;
				end else begin
					next_state <= IF;
				end
			end
			IW: begin
				if (Inst_Valid) begin
					next_state <= ID;
				end else begin
					next_state <= IW;
				end
			end
			ID: begin
				if (|IR) begin
					next_state <= EX;
				end else begin
					next_state <= IF;
				end
			end
			EX: begin
			 	if (R_Type | I_Type_calc | J_Type & opcode[0]) begin
			 		next_state <= WB;
			 	end else if (I_Type_mr) begin
			 		next_state <= LD;
			 	end else if (I_Type_mw) begin
					next_state <= ST;
				end else begin
			 		next_state <= IF;
			 	end	
			end
			ST: begin
				if (Mem_Req_Ready) begin
					next_state <= IF;
				end else begin
					next_state <= ST;
				end
			end
			LD: begin
				if (Mem_Req_Ready) begin
					next_state <= RDW;
				end else begin
					next_state <= LD;
				end
			end
			RDW: begin
				if (Read_data_Valid) begin
					next_state <= WB;
				end else begin
					next_state <= RDW;
				end
			end
			WB: next_state <= IF;
			default: next_state <= INIT;
		endcase
	end

	assign Inst_Req_Valid	= current_state[isIF];
	assign Inst_Ready		= current_state[isIW] || current_state[isINIT];
	assign MemWrite			= current_state[isST];
	assign MemRead			= current_state[isLD];
	assign Read_data_Ready	= current_state[isRDW] || current_state[isINIT];

	//PC
	assign PC_branch = PC + {SignExtend[29:0], 2'b00};
	assign Branch_or_not = (Zero ^ (
		REGIMM & ~IR[16] 
		| I_Type_b & (opcode[0] ^ (opcode[1] & |RF_rdata1))
	)) & Branch;
	always @(posedge clk) begin
		if (rst) begin
			PC <= 32'd0;
		end else if (current_state[isIW] && Inst_Valid) begin
			PC <= PC + 4;
		end else if (current_state[isEX]) begin
			if (R_Type_jump) begin
				PC <= RF_rdata1;
			end else if (J_Type) begin
				PC <= {IR[25:0], 2'b00};	
			end else if (Branch_or_not) begin
				PC <= PC_branch;
			end
		end
	end

	//for R_Type_jump | J_Type
	always @(posedge clk) begin
		if (current_state[isID]) begin
			PC_normal <= PC;
		end
	end

	//IR
	assign {opcode,rs,rt,rd,shamt,func} = IR;
	always @(posedge clk) begin
		if (current_state[isIW] && Inst_Valid) begin
			IR <= Instruction;
		end
	end

	//Controller
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
	assign MemtoReg	= I_Type_mr;
	assign ALUEn	= R_Type_calc | REGIMM | I_Type_b | I_Type_calc | I_Type_mr | I_Type_mw;
	assign ShiftEn	= R_Type_shift;
	assign ALUSrc	= I_Type_calc | I_Type_mr | I_Type_mw;
	assign ShiftSrc = func[2];
	assign RF_wen	= (current_state[isWB]) & (
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
		if (current_state[isID]) begin
			RF_rdata1 <= RF_rdata1_wire;
			RF_rdata2 <= RF_rdata2_wire;
		end
	end

	//Extend
	assign ExtendedImm = opcode[5:2] == 4'b0011 ? ZeroExtend : SignExtend;
	always @(posedge clk) begin
		if (current_state[isID]) begin
			SignExtend <= {{(16){IR[15]}}, IR[15:0]};
			ZeroExtend <= {16'b0, IR[15:0]};
		end
	end

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
		if (current_state[isEX]) begin
			Result <= {(32){ALUEn}} & ALUResult
					| {(32){ShiftEn}} & ShifterResult;
		end
	end

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
	always @(posedge clk) begin
		if (current_state[isRDW] && Read_data_Valid) begin
			MDR <= Read_data;
		end
	end

	//Write Back
	assign Read_data_shifted 	= MDR >> {Result[1:0], 3'b0};
	assign Read_data_masked 	= Read_data_shifted & {{(16){opcode[1]}}, {(8){opcode[0]}}, {(8){1'b1}}}
								| {(32){~opcode[2] & Read_data_sign_bit}} & ~{{(16){opcode[1]}}, {(8){opcode[0]}}, {(8){1'b1}}};
	assign Read_data_sign_bit 	= Read_data_shifted[opcode[1:0]==2'b01 ? 15 : 7];
	assign Read_data_unaligned 	= {(32){~opcode[2]}} & (
									(MDR << {~Result[1:0], 3'b0}) | RF_rdata2 & ({(32){1'b1}} >> { Result[1:0], 3'b0})
								)|{(32){opcode[2]}} & (
									(MDR >> { Result[1:0], 3'b0}) | RF_rdata2 & ({(32){1'b1}} << {~Result[1:0], 3'b0})
								);
	assign RF_wdata = {(32){MemtoReg & (~opcode[1] |  opcode[0])}} & Read_data_masked
					| {(32){MemtoReg & ( opcode[1] & ~opcode[0])}} & Read_data_unaligned
					| {(32){R_Type_mov}} & RF_rdata1 
					| {(32){R_Type_jump | J_Type}} & PC_normal + 4
					| {(32){I_Type_calc & (&opcode[3:0])}} & {IR[15:0], 16'd0}
					| {(32){R_Type_calc | R_Type_shift | I_Type_calc & ~&opcode[3:0]}} & Result;

endmodule
