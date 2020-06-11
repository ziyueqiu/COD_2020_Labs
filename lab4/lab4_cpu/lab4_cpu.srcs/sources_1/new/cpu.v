module cpu(
    input clk,
    input rst
);
// PC
wire [31 : 0] new_addr, cur_addr;
wire PCwe;
// Mem
wire [31 : 0] MemAddr;
wire [31 : 0] MemData, instruction;
//wire [31 : 0] MemWriteData;
wire [31 : 0] MemDataReg;
// instruction
wire [5 : 0] Op;
wire [15 : 0] imm;
wire [5 : 0] funct;
// Control
wire PCWriteCond, PCWrite, lorD, ALUSrcA, MemWrite, RegWrite;
wire MemtoReg, IRWrite, RegDst, MemRead;
wire [1 : 0] PCSource, ALUOp, ALUSrcB;
// Registers
wire [4 : 0] ReadReg1, ReadReg2, Reg3, WriteReg;
wire [31 : 0] RegWriteData;
wire [31 : 0] ReadData1, ReadData2;
wire [31 : 0] A, B;
// ALU
wire zero;
wire [31 : 0] ALUA, ALUB;
wire [31 : 0] ALUresult;
wire [31 : 0] ALUOut;
wire [2 : 0] ALUm;
wire [31 : 0] extendImm, SHIFT1;
wire [31 : 0] JumpAddr;

// 信号 从左到右连线
assign PCwe = (zero&PCWriteCond)|PCWrite;
assign Op = instruction[31 : 26];
assign ReadReg1 = instruction[25 : 21];
assign ReadReg2 = instruction[20 : 16];
assign Reg3 = instruction[15 : 11];
assign imm = instruction[15 : 0];
assign funct = instruction[5 : 0];
assign SHIFT1 = {extendImm[29 : 0], 2'b00};
assign JumpAddr = {cur_addr[31 : 28], instruction[25 : 0], 2'b00};
/*
module PC(
    input clk,
    input rst,
    input PCwe,
    input [31:0] new_addr,
    output reg [31:0] cur_addr
);*/
PC pc(
    .clk(clk),
    .rst(rst),
    .PCwe(PCwe),
    .new_addr(new_addr),
    .cur_addr(cur_addr)
);

/*
module Instruction(
    input [31 : 0] MemData,
    input IRWrite,
    output reg [31 : 0] instruction
);*/
Instruction IR(
    .MemData(MemData),
    .clk(clk),
    .IRWrite(IRWrite),
    .instruction(instruction)
);

/*
module Control(
    input [5 : 0] Op,
    input clk,
    output PCWriteCond,
    output [1 : 0] PCSource,
    output PCWrite,
    output [1 : 0] ALUOp,
    output lorD,
    output [1 : 0] ALUSrcB,
    output ALUSrcA,
    output MemWrite, RegWrite, MemRead,
    output MemtoReg,
    output IRWrite, RegDst
);
*/
Control control(
    .Op(Op),
    .clk(clk),
    .rst(rst),
    .PCWriteCond(PCWriteCond),
    .PCSource(PCSource),
    .PCWrite(PCWrite),
    .ALUOp(ALUOp),
    .lorD(lorD),
    .ALUSrcB(ALUSrcB),
    .ALUSrcA(ALUSrcA),
    .MemWrite(MemWrite),
    .RegWrite(RegWrite),
    .MemRead(MemRead),
    .MemtoReg(MemtoReg),
    .IRWrite(IRWrite),
    .RegDst(RegDst)
);

/*
module Mux32(
    input control,
    input [31:0] in1, in0,
    output [31:0] out
);
*/
Mux32 mux0(
    .control(lorD),
    .in0(cur_addr),
    .in1(ALUOut),
    .out(MemAddr)
);


// DataMemory 512*32
dist_mem_gen_0 dist0 (
  .a(MemAddr[10 : 2]),  // input wire [8 : 0]  地址
  .d(B),     // input wire [31 : 0] d 写数据
  .clk(clk),         // input wire clk
  .we(MemWrite),     // input wire we 写使能
  .spo(MemData)  // output wire [31 : 0] spo 读数据
);

/*
module Mux5(
    input control,
    input [4:0] in1, in0,
    output [4:0] out
);
*/
Mux5 mux1(
    .control(RegDst),
    .in0(ReadReg2),
    .in1(Reg3),
    .out(WriteReg)
);

/*
module Register(
    input clk,
    input [31 : 0] in,
    output reg [31 : 0] out
);*/
Register memdata(
    .clk(clk),
    .in(MemData),
    .out(MemDataReg)
);

/*
module Mux32(
    input control,
    input [31:0] in1, in0,
    output [31:0] out
);
*/
Mux32 mux2(
    .control(MemtoReg),
    .in0(ALUOut),
    .in1(MemDataReg),
    .out(RegWriteData)
);

/*
module Registers				//32 x WIDTH寄存器堆
#(parameter WIDTH = 32) 	        //数据宽度
(
    input clk,						//时钟（上升沿有效）
    input [4:0] ra0,				//读端口0地址
    output reg [WIDTH-1:0] rd0,    	    //读端口0数据
    input [4:0] ra1, 				//读端口1地址
    output reg [WIDTH-1:0] rd1,      	//读端口1数据
    input [4:0] wa, 				//写端口地址
    input we,				    	//写使能，高电平有效
    input [WIDTH-1:0] wd 	    	//写端口数据
);*/
Registers registers(
    .clk(clk),
    .ra0(ReadReg1),
    .rd0(ReadData1),
    .ra1(ReadReg2),
    .rd1(ReadData2),
    .wa(WriteReg),
    .we(RegWrite),
    .wd(RegWriteData)
);

Register a(
    .clk(clk),
    .in(ReadData1),
    .out(A)
);

Register b(
    .clk(clk),
    .in(ReadData2),
    .out(B)
);

Mux32 mux3(
    .control(ALUSrcA),
    .in0(cur_addr),
    .in1(A),
    .out(ALUA)
);

/*
module Sign_extend(
    input [15:0] imm,
    output [31:0] extendImm
);*/
Sign_extend sign_extend(
    .imm(imm),
    .extendImm(extendImm)
);

/*
module Mux4_32(
    input [1:0] control,
    input [31:0] in11, in10, in01, in00,
    output [31:0] out
);
*/
Mux4_32 mux4(
    .control(ALUSrcB),
    .in00(B),
    .in01(32'd4),
    .in10(extendImm),
    .in11(SHIFT1),
    .out(ALUB)
);

/*
module ALUControl(
    input Op1, Op0,
    input [5 : 0] funct,
    output reg [2 : 0] ALUOp
);
*/
ALUControl alu_control(
    .Op1(ALUOp[1]),
    .Op0(ALUOp[0]),
    .funct(funct),
    .ALUOp(ALUm)
);

/*
module ALU
#(parameter WIDTH = 32) 	//数据宽度
(output reg [WIDTH-1:0] y, 		//运算结果
output reg zf, 					//零标志
output reg cf, 					//进位/借位标志
output reg of, 					//溢出标志
input [WIDTH-1:0] a, b,		//两操作数
input [2:0] m		    	//操作类型
);*/
ALU alu(
    .y(ALUresult),
    .zf(zero),
    .cf(),
    .of(),
    .a(ALUA),
    .b(ALUB),
    .m(ALUm)
);

Register aluout(
    .clk(clk),
    .in(ALUresult),
    .out(ALUOut)
);

/*
module Mux3_32(
    input [1:0] control,
    input [31:0] in10, in01, in00,
    output [31:0] out
);*/
Mux3_32 mux5(
    .control(PCSource),
    .in00(ALUresult),
    .in01(ALUOut),
    .in10(JumpAddr),
    .out(new_addr)
);
endmodule