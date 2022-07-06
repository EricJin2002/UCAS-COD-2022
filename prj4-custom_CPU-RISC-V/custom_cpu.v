`timescale 10ns / 1ns
//`include "reg_file/reg_file.v"
//`include "alu/alu.v"
//`include "shifter/shifter.v"
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

	reg  [31:0]	PC_reg;
	wire		Branch_or_not;
	reg  [31:0]	PC_normal;

	reg  [31:0] IR;
	wire [6:0]  funct7;
	wire [4:0]  rs2;
	wire [4:0]  rs1;
	wire [2:0]  funct3;
	wire [4:0]  rd;
	wire [6:0]  opcode;
	wire [31:0] imm;

	wire OP_IMM;
	wire LUI;
	wire AUIPC;
	wire OP;
	wire JAL;
	wire JALR;
	wire BRANCH;
	wire LOAD;
	wire STORE;

	wire R_Type;
	wire I_Type;
	wire S_Type;
	wire B_Type;
	wire U_Type;
	wire J_Type;

	wire Branch,MemtoReg,ALUEn,ShiftEn,MULEn,ALUSrc,ShiftSrc,RF_wen;

	wire [31:0] RF_rdata1_wire;
	wire [31:0] RF_rdata2_wire;
	reg  [31:0] RF_rdata1;
	reg  [31:0] RF_rdata2;
	wire [4:0]	RF_waddr;
	wire [31:0]	RF_wdata;

	wire [2:0]	ALUop;
	wire [1:0]	Shiftop;
	wire 		Overflow,CarryOut,Zero;
	wire [31:0]	ALUResult,ShifterResult,MULResult;
	reg  [31:0]	Result;

	reg  [31:0]	MDR;

	wire [31:0]	Read_data_shifted;
	wire [31:0]	Read_data_masked;
	wire 		Read_data_sign_bit;
	wire [31:0]	Read_data_unaligned;


	assign inst_retire [69:69] = RF_wen;
	assign inst_retire [68:64] = RF_waddr;
	assign inst_retire [63:32] = RF_wdata;
	assign inst_retire [31: 0] = PC_normal;


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
		end else if (current_state[isEX] && LOAD) begin
			mr_cnt <= mr_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_2 = mr_cnt;

	reg  [31:0] mw_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			mw_cnt <= 32'd0;
		end else if (current_state[isEX] && STORE) begin
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
		end else if (current_state[isID] && BRANCH) begin
			branch_inst_cnt <= branch_inst_cnt + 32'd1;
		end
	end
	assign cpu_perf_cnt_9 = branch_inst_cnt;

	reg  [31:0] jump_inst_cnt;
	always @ (posedge clk) begin
		if (rst) begin
			jump_inst_cnt <= 32'd0;
		end else if (current_state[isID] && (JAL || JALR)) begin
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
			ID: next_state <= EX;
			EX: begin
			 	if (R_Type | I_Type & ~LOAD | U_Type | J_Type) begin
			 		next_state <= WB;
			 	end else if (LOAD) begin
			 		next_state <= LD;
			 	end else if (S_Type) begin
					next_state <= ST;
				end else if (B_Type) begin
			 		next_state <= IF;
				end	else begin
					next_state <= INIT;
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
	assign Branch_or_not = (Zero ^ funct3[2] ^ funct3[0]) & Branch;
	always @(posedge clk) begin
		if (rst) begin
			PC_reg <= 32'd0;
		end /*else if (current_state[isIW] && Inst_Valid) begin
			PC_reg <= PC_reg + 4;
		end */else if (current_state[isEX]) begin
			if (JAL) begin
				PC_reg <= PC_reg + imm;
			end else if (JALR) begin
				PC_reg <= (RF_rdata1 + imm) & {~31'b0, 1'b0};
			end else if (Branch_or_not) begin
				PC_reg <= PC_reg + imm;
			end else begin
				PC_reg <= PC_reg + 4;
			end
		end
	end
	assign PC = PC_reg;

	always @(posedge clk) begin
		if (current_state[isID]) begin
			PC_normal <= PC_reg;
		end
	end

	//IR
	always @(posedge clk) begin
		if (current_state[isIW] && Inst_Valid) begin
			IR <= Instruction;
		end
	end

	//ID
	assign {funct7, rs2, rs1, funct3, rd, opcode} = IR;
	assign imm = {
		{ //31:12
			U_Type | J_Type ?
			{ //31:12
				{ //31:20
					U_Type ? {IR[31],IR[30:20]} : {(12){IR[31]}}
				}, 
				IR[19:12] //19:12
			} : {(20){IR[31]}}
		}, 
		{ //11:0
			U_Type ? 12'b0 :
			{ //11:0
				{ //11
					B_Type ? IR[7] : J_Type ? IR[20] : IR[31]
				}, 
				IR[30:25], //10:5
				{ //4:1
					I_Type | J_Type ? IR[24:21] : IR[11:8]
				},
				{ //0
					I_Type ? IR[20] : S_Type ? IR[7] : 1'b0
				}
			}
		}
	};

	//Controller
	assign OP_IMM	= opcode[6:0] == 7'b0010011;	
	assign LUI		= opcode[6:0] == 7'b0110111;
	assign AUIPC	= opcode[6:0] == 7'b0010111;
	assign OP		= opcode[6:0] == 7'b0110011;
	assign JAL		= opcode[6:0] == 7'b1101111;
	assign JALR		= opcode[6:0] == 7'b1100111;
	assign BRANCH	= opcode[6:0] == 7'b1100011;
	assign LOAD		= opcode[6:0] == 7'b0000011;
	assign STORE	= opcode[6:0] == 7'b0100011;

	assign R_Type = OP;
	assign I_Type = OP_IMM | JALR | LOAD;
	assign S_Type = STORE;
	assign B_Type = BRANCH;
	assign U_Type = LUI | AUIPC;
	assign J_Type = JAL;

	assign Branch 	= B_Type;
	assign MemtoReg	= LOAD;
	assign ALUEn	= (OP & ~funct7[0] | OP_IMM) & (funct3[1] | ~funct3[0]) | JALR | LOAD | S_Type | B_Type;
	assign ShiftEn	= (OP & ~funct7[0] | OP_IMM) & (~funct3[1] & funct3[0]);
	assign MULEn	= OP & funct7[0];
	assign ALUSrc	= I_Type | S_Type;
	assign ShiftSrc = I_Type | S_Type;
	assign RF_wen	= current_state[isWB] & (J_Type | I_Type | R_Type | U_Type);

	//Registers
	reg_file Registers(
		.clk	(clk),
		.waddr	(RF_waddr),
		.raddr1	(rs1),
		.raddr2	(rs2),
		.wen	(RF_wen),
		.wdata	(RF_wdata),
		.rdata1	(RF_rdata1_wire),
		.rdata2	(RF_rdata2_wire)
	);

	assign RF_waddr = rd;
	always @(posedge clk) begin
		if (current_state[isID]) begin
			RF_rdata1 <= RF_rdata1_wire;
			RF_rdata2 <= RF_rdata2_wire;
		end
	end

	//ALU
	alu ALU(
		.A			(RF_rdata1),
		.B			(ALUSrc ? imm : RF_rdata2),
		.ALUop		(ALUop),
		.Overflow	(Overflow),
		.CarryOut	(CarryOut),
		.Zero		(Zero),
		.Result		(ALUResult)
	);
	assign ALUop = {(3){OP_IMM | OP}} & {
		(funct3 == 3'b100)									//xor
			| (funct3 == 3'b010) 							//slt
			| (funct3 == 3'b000 && funct7[5] && opcode[5]), //sub
		~funct3[2],											//and, or, xor
		funct3[1] & ~(funct3[0] & funct3[2])				//or, slt, sltu
	} | {(3){STORE | LOAD | JALR}} & 3'b010 								//add 010
	  | {(3){BRANCH}} & {~funct3[1], 1'b1, funct3[2]};		//sub 110 slt 111 sltu 011

	//Shifter
	shifter Shifter(
		.A			(RF_rdata1),
		.B			(ShiftSrc ? imm[4:0] : RF_rdata2[4:0]),
		.Shiftop	(Shiftop),
		.Result		(ShifterResult)
	);
	assign Shiftop = {funct3[2],funct7[5]};

	//MUL
	assign MULResult = RF_rdata1 * RF_rdata2;

	//Result
	always @(posedge clk) begin
		if (current_state[isEX]) begin
			Result <= {(32){ALUEn}}		& ALUResult
					| {(32){ShiftEn}} 	& ShifterResult
					| {(32){MULEn}} 	& MULResult;
		end
	end

	//Write
	assign Address 		= Result & ~32'b11;
	assign Write_data	= RF_rdata2 << {Result[1:0], 3'b0};
	assign Write_strb	= { ( Result[1] | funct3[1]) & ( Result[0] | funct3[0] | funct3[1]),
							( Result[1] | funct3[1]) & (~Result[0] | funct3[0] | funct3[1]),
						   	(~Result[1] | funct3[1]) & ( Result[0] | funct3[0] | funct3[1]),
						    (~Result[1] | funct3[1]) & (~Result[0] | funct3[0] | funct3[1]) 
						};
	
	//Read
	always @(posedge clk) begin
		if (current_state[isRDW] && Read_data_Valid) begin
			MDR <= Read_data;
		end
	end

	//Write Back
	assign Read_data_shifted 	= MDR >> {Result[1:0], 3'b0};
	assign Read_data_masked 	= Read_data_shifted & {{(16){funct3[1]}}, {(8){funct3[0] | funct3[1]}}, {(8){1'b1}}}
								| {(32){~funct3[2] & Read_data_sign_bit}} & ~{{(16){funct3[1]}}, {(8){funct3[0] | funct3[1]}}, {(8){1'b1}}};
	assign Read_data_sign_bit 	= Read_data_shifted[funct3[1:0]==2'b01 ? 15 : 7];
	assign RF_wdata = {(32){LUI}}			& imm
					| {(32){AUIPC}}			& PC_normal + imm
					| {(32){JAL | JALR}}	& PC_normal + 4
					| {(32){LOAD}}			& Read_data_masked
					| {(32){OP | OP_IMM}}	& Result;

endmodule
