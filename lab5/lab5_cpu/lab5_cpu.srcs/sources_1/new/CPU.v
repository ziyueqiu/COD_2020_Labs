module CPU(
    input clk,
    input rst
);
parameter ZERO = 0;
// IF
wire [31 : 0] new_addr, cur_addr;
wire [31 : 0] IF_PCadd;
wire [31 : 0] IF_instruction;
// ID
wire [31 : 0] ID_PCadd, ID_instruction;
wire [5 : 0] op;
wire ID_RegDst, ID_ALUOp1, ID_ALUOp0, ID_ALUSrc;
wire ID_Branch, ID_MemRead, ID_MemWrite;
wire ID_RegWrite, ID_MemtoReg;
wire [4 : 0] ID_Rs, ID_Rt, ID_Rd;
wire [15 : 0] ID_imm;
wire [31 : 0] ID_extendImm;
wire [31 : 0] ID_RegData0, ID_RegData1;
wire [8 : 0] Control_Signal, ID_Signal;
wire jump; // 信号
wire [31 : 0] jumpAddr, MUXtonewaddr;
// EX
wire EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc;
wire EX_Branch, EX_MemRead, EX_MemWrite;
wire EX_RegWrite, EX_MemtoReg;
wire [1 : 0] ForwardA, ForwardB;
wire ForwardC;
wire [31 : 0] ALUA, ALUB, MUXtoALUB, EX_ALUy;
wire [31 : 0] EX_RegData0, EX_RegData1;
wire EX_zero;
wire [2 : 0] m; // ALUop
wire [4 : 0] EX_Rtd, EX_Rt, EX_Rs, EX_Rd;
wire [31 : 0] EX_extendImm, EX_SHIFT, EX_PCadd, PCBranch;
// MEM
wire MEM_Branch, MEM_MemRead, MEM_MemWrite;
wire MEM_RegWrite, MEM_MemtoReg;
wire MEM_zero;
wire [31 : 0] MEM_ALUy, MEM_RegData1;
wire [4 : 0] MEM_Rd;
wire [31 : 0] MEM_Data, MUXtoData;
// WB
wire WB_RegWrite, WB_MemtoReg;
wire [31 : 0] WB_MUXtoReg;
wire [31 : 0] WB_Data, WB_ALUy;
wire [4 :0] WB_Rd;

wire stall;
wire Clrn1, Clrn2;
wire PCSrc;

// assign
assign IF_PCadd = cur_addr+4;
assign op = ID_instruction[31 : 26];
assign ID_Rs = ID_instruction[25 : 21];
assign ID_Rt = ID_instruction[20 : 16];
assign ID_Rd = ID_instruction[15 : 11];
assign ID_imm = ID_instruction[15 : 0];
assign EX_SHIFT = {EX_extendImm[29 : 0], 2'b00};
assign PCBranch = EX_PCadd + EX_SHIFT;
assign Control_Signal = {ID_RegDst, ID_ALUOp1, ID_ALUOp0, ID_ALUSrc,
                        ID_Branch, ID_MemRead, ID_MemWrite,
                        ID_RegWrite, ID_MemtoReg};
//assign PCSrc = EX_Branch & EX_zero;
assign PCSrc = (EX_Branch & EX_zero)? 1 : 0;
assign jump = (ID_instruction[31:26] == 6'b000010)?1:0;
assign jumpAddr = {ID_PCadd[31:28], ID_instruction[25 : 0],2'b00};
/*
module PC(
    input clk,
    input rst,
    input stall,
    input [31:0] new_addr,
    output reg [31:0] cur_addr
);
*/
PC pc(
  .clk(clk),
  .rst(rst),
  .stall(stall),
  .new_addr(new_addr),
  .cur_addr(cur_addr)
);

/*
module Mux2_32(
    input control,
    input [31:0] in1, in0,
    output [31:0] out
);
*/

Mux2_32 mux0(
  .control(PCSrc),
  .in0(MUXtonewaddr),
  .in1(PCBranch),
  .out(new_addr)
);
Mux2_32 mux8(
  .control(jump),
  .in0(IF_PCadd),
  .in1(jumpAddr),
  .out(MUXtonewaddr)
);

// InstructionMemory 256*32
InstructionMemory rom(
    .a(cur_addr[9:2]),          // 读地址
    .spo(IF_instruction)      // 读数据
);

/*
module IF_ID_Registers(
    input clk,
    input [31 : 0] IF_PCadd,
    input [31 : 0] IF_instruction, 
    input Clrn, 
    output reg [31 : 0] ID_PCadd,
    output reg [31 : 0] ID_instruction
);
*/
IF_ID_Registers R0(
    .clk(clk),
    .Clrn(Clrn1),
    .stall(stall),
    .IF_PCadd(IF_PCadd),
    .IF_instruction(IF_instruction),
    .ID_PCadd(ID_PCadd),
    .ID_instruction(ID_instruction)
);

/*
module Control(
    input [5:0] instruction,
    output reg EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
    output reg M_Branch, M_MemRead, M_MemWrite,
    output reg WB_RegWrite, WB_MemtoReg
);
*/
Control control(
    .instruction(op),
    .EX_RegDst(ID_RegDst),
    .EX_ALUOp1(ID_ALUOp1),
    .EX_ALUOp0(ID_ALUOp0),
    .EX_ALUSrc(ID_ALUSrc),
    .M_Branch(ID_Branch),
    .M_MemRead(ID_MemRead),
    .M_MemWrite(ID_MemWrite),
    .WB_RegWrite(ID_RegWrite),
    .WB_MemtoReg(ID_MemtoReg)
);

/*
module Sign_extend(
    input [15:0] imm,
    output [31:0] extendImm
);
*/
Sign_extend signextend(
    .imm(ID_imm),
    .extendImm(ID_extendImm)
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
);
*/
Registers registers(
    .clk(clk),
    .ra0(ID_Rs),
    .rd0(ID_RegData0),
    .ra1(ID_Rt),
    .rd1(ID_RegData1),
    .wa(WB_Rd),
    .we(WB_RegWrite),
    .wd(WB_MUXtoReg)
);

/*
module Mux2_9(
    input control,
    input [8:0] in1, in0,
    output [8:0] out
);
*/
Mux2_9 mux2(
    .control(stall),
    .in0(Control_Signal),
    .in1(ZERO),
    .out(ID_Signal)
);


/*
module HazardDetectionUnit(
    input EX_MemRead, EX_Branch, EX_zero,
    input [4 : 0] ID_Rs, ID_Rt, 
    input [4 : 0] EX_Rtd, 
    output reg stall, Clrn
);
);*/
HazardDetectionUnit hazard(
    .EX_MemRead(EX_MemRead),
    .EX_Branch(EX_Branch),
    .EX_zero(EX_zero),
    .ID_Rs(ID_Rs),
    .ID_Rt(ID_Rt),
    .EX_Rtd(EX_Rtd),
    .jump(jump),
    .stall(stall),
    .rst(rst),
    .Clrn1(Clrn1),
    .Clrn2(Clrn2)
);

/*
module ID_EX_Registers(
    input clk,
    input Clrn,
    input [8 : 0] ID_Signal, 
    input [31 : 0] ID_RegData0, ID_RegData1,
    input [31 : 0] extendImm,
    input [4 : 0] ID_Rs, ID_Rt, ID_Rd,
    output reg EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc,
            EX_Branch, EX_MemRead, EX_MemWrite,
            EX_RegWrite, EX_MemtoReg,
    output reg [31 : 0] EX_RegData0, EX_RegData1, 
    output reg [31 : 0] EX_extendImm,
    output reg [4 : 0] EX_Rs, EX_Rt, EX_Rd
);
*/
ID_EX_Registers R1(
    .clk(clk),
    .Clrn(Clrn2),
    .ID_Signal(ID_Signal),
    .ID_RegData1(ID_RegData1),
    .ID_RegData0(ID_RegData0),
    .ID_PCadd(ID_PCadd),
    .extendImm(ID_extendImm),
    .ID_Rs(ID_Rs),
    .ID_Rt(ID_Rt),
    .ID_Rd(ID_Rd),
    .EX_RegDst(EX_RegDst),
    .EX_ALUOp1(EX_ALUOp1),
    .EX_ALUOp0(EX_ALUOp0),
    .EX_ALUSrc(EX_ALUSrc),
    .EX_Branch(EX_Branch),
    .EX_MemRead(EX_MemRead),
    .EX_MemWrite(EX_MemWrite),
    .EX_RegWrite(EX_RegWrite),
    .EX_MemtoReg(EX_MemtoReg),
    .EX_RegData1(EX_RegData1),
    .EX_RegData0(EX_RegData0),
    .EX_extendImm(EX_extendImm),
    .EX_PCadd(EX_PCadd),
    .EX_Rs(EX_Rs),
    .EX_Rt(EX_Rt),
    .EX_Rd(EX_Rd)
);

/*
module ForwardingUnit(
    input [4 : 0] ID_EX_Rt, ID_EX_Rs,
    input [4 : 0] EX_MEM_Rd, 
    input [4 : 0] MEM_WB_Rd, 
    input EX_MEM_RegWrite, MEM_WB_RegWrite,
    output reg [1 : 0] ForwardA, ForwardB
);
*/
ForwardingUnit forwarding(
    .ID_EX_Rt(EX_Rt),
    .ID_EX_Rs(EX_Rs),
    .EX_MEM_Rd(MEM_Rd),
    .MEM_WB_Rd(WB_Rd),
    .EX_MEM_RegWrite(MEM_RegWrite),
    .MEM_WB_RegWrite(WB_RegWrite),
    .EX_MEM_MemWrite(MEM_MemWrite),
    .ForwardA(ForwardA),
    .ForwardB(ForwardB),
    .ForwardC(ForwardC)
);

/*
module Mux3_32(
    input [1:0] control,
    input [31:0] in10, in01, in00,
    output [31:0] out
);
*/
Mux3_32 mux3(
    .control(ForwardA),
    .in10(MEM_ALUy),
    .in01(WB_MUXtoReg),
    .in00(EX_RegData0),
    .out(ALUA)
);

Mux3_32 mux4(
    .control(ForwardB),
    .in10(MEM_ALUy),
    .in01(WB_MUXtoReg),
    .in00(EX_RegData1),
    .out(MUXtoALUB)
);

Mux2_32 mux5(
  .control(EX_ALUSrc),
  .in0(MUXtoALUB),
  .in1(EX_extendImm),
  .out(ALUB)
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
);
*/
ALU alu(
    .y(EX_ALUy),
    .zf(EX_zero),
    .cf(),
    .of(),
    .a(ALUA),
    .b(ALUB),
    .m(m)
);

/*
module ALUControl(
    input Op1, Op0,
    input [5 : 0] funct,
    output reg [2 : 0] ALUOp
);
*/
ALUControl alucontrol(
    .Op1(EX_ALUOp1),
    .Op0(EX_ALUOp0),
    .funct(EX_extendImm[5 : 0]),
    .ALUOp(m)
);

Mux2_5 mux6(
  .control(EX_RegDst),
  .in0(EX_Rt),
  .in1(EX_Rd),
  .out(EX_Rtd)
);

/*
EX_MEM_Registers(
    input clk,
    input EX_Branch, EX_MemRead, EX_MemWrite,
        EX_RegWrite, EX_MemtoReg,
    input zero,
    input [31 : 0] ALUy, EX_RegData1
    input [4 : 0] EX_Rtd,
    output reg MEM_Branch, MEM_MemRead, MEM_MemWrite,
        MEM_RegWrite, MEM_MemtoReg,
    output reg MEM_zero,
    output reg [31 : 0] MEM_ALUy, MEM_RegData1
    output reg [4 : 0] MEM_Rd,
);
*/
EX_MEM_Registers R2(
    .clk(clk),
    .EX_Branch(EX_Branch),
    .EX_MemRead(EX_MemRead),
    .EX_MemWrite(EX_MemWrite),
    .EX_RegWrite(EX_RegWrite),
    .EX_MemtoReg(EX_MemtoReg),
    .EX_RegData1(EX_RegData1),
    //.zero(EX_zero),
    .ALUy(EX_ALUy),
    .EX_Rtd(EX_Rtd),
    .MEM_Branch(MEM_Branch),
    .MEM_MemRead(MEM_MemRead),
    .MEM_MemWrite(MEM_MemWrite),
    .MEM_RegWrite(MEM_RegWrite),
    .MEM_MemtoReg(MEM_MemtoReg),
    //.MEM_zero(MEM_zero),
    .MEM_ALUy(MEM_ALUy),
    .MEM_RegData1(MEM_RegData1),
    .MEM_Rd(MEM_Rd)
);

Mux2_32 mux9(
    .control(ForwardC),
    .in0(MEM_RegData1),
    .in1(WB_MUXtoReg),
    .out(MUXtoData)
);

// DataMemory 256*32
DataMemory ram (
  .a(MEM_ALUy[9 : 2]),  // input wire [7 : 0]  地址
  .d(MUXtoData),     // input wire [31 : 0] d 写数据
  .clk(clk),         // input wire clk
  .we(MEM_MemWrite),     // input wire we 写使能
  .spo(MEM_Data)  // output wire [31 : 0] spo 读数据
);

/*
MEM_WB_Registers(
    input clk,
    input MEM_RegWrite, MEM_MemtoReg,
    input [31 : 0] MEM_Data,
    input [31 : 0] MEM_ALUy,
    input [4 : 0] MEM_Rd,
    output reg WB_RegWrite, WB_MemtoReg,
    output reg [31 : 0] WB_ALUy, WB_Data,
    output reg [4 : 0] WB_Rd
);
*/
MEM_WB_Registers R3(
    .clk(clk),
    .MEM_RegWrite(MEM_RegWrite),
    .MEM_MemtoReg(MEM_MemtoReg),
    .MEM_Data(MEM_Data),
    .MEM_ALUy(MEM_ALUy),
    .MEM_Rd(MEM_Rd),
    .WB_RegWrite(WB_RegWrite),
    .WB_MemtoReg(WB_MemtoReg),
    .WB_ALUy(WB_ALUy),
    .WB_Data(WB_Data),
    .WB_Rd(WB_Rd)
);

Mux2_32 mux7(
  .control(WB_MemtoReg),
  .in0(WB_ALUy),
  .in1(WB_Data),
  .out(WB_MUXtoReg)
);

endmodule