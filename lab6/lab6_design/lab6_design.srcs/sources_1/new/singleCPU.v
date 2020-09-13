module cpu_one_cycle( // 单周期 CPU
    input clk,        // 时钟（上升沿有效）
    input rst,        // 异步复位，高电平有效
    input I_step, I_flag, // btn
    input [15 : 0] I_data,
    output [15 : 0] O_led
);
localparam m = 3'b0;
// PC
wire [31 : 0] new_addr, cur_addr, PCadd;
wire [31 : 0] insturction;        // insturction
wire [31 : 0] ALUresult0;
wire [27 : 0] Shift0;
wire [31 : 0] JumpAddr, Mux4out, Mux6out;
wire [33 : 0] Shift1;
wire [5 : 0] funct;
wire [4 : 0] shamt;
// 信号
wire [5:0] op;
wire ShamtSignal;
wire RegDst, Jump, Branch, MemtoReg, MemWrite, ALUSrc, RegWrite;
// Registers
wire [4 : 0] ReadReg1, ReadReg2, ReadReg3, WriteReg;
wire [31 : 0] ReadData1, ReadData2, WriteData;
// signExtend
wire [15 : 0] imm;
wire [31 : 0] extendImm;
// 纯拓展
wire [31 : 0] extendShamt;
// ALU
wire [31 : 0] ALUin2, ALUresult1;
wire zero;
//wire [2 : 0] m;
// Mem
wire [31 : 0] MemReadData;
// ALUControl
wire Op1, Op0;
wire [2:0] ALUOp;

// assign
assign op = insturction[31 : 26];
assign ReadReg1 = insturction[25 : 21];
assign ReadReg2 = insturction[20 : 16];
assign ReadReg3 = insturction[15 : 11];
assign imm = insturction[15 : 0];
assign funct = insturction[5 : 0];
assign shamt = insturction[10 : 6];
assign extendShamt = {27'b0, shamt};
//assign m = 3'b000;
assign PCadd = cur_addr + 4;
assign Shift0 = {insturction[25 : 0], 2'b00};
assign JumpAddr = {PCadd[31 : 28], Shift0};
assign Shift1 = {extendImm, 2'b00};

Mux32 mux6(
    .control(ShamtSignal),
    .in0(ReadData1),
    .in1(extendShamt),
    .out(Mux6out)
);

Mux32 mux4(
    .control(Branch&zero),
    .in0(PCadd),
    .in1(ALUresult0),
    .out(Mux4out)
);

Mux32 mux5(
    .control(Jump),
    .in0(Mux4out),
    .in1(JumpAddr),
    .out(new_addr)
);

ALU alu0(
    .y(ALUresult0),
    .zf(),
    .cf(),
    .of(),
    .a(PCadd),
    .b(Shift1[31 : 0]),
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
    .Op1(Op1),
    .Op0(Op0),
    .funct(funct),
    .ALUOp(ALUOp),
    .ShamtSignal(ShamtSignal)
);


/*
module PC(
    input clk,
    input rst,
    input [31:0] new_addr,
    output reg [31:0] cur_addr
);*/
PC pc(
    .clk(clk),
    .rst(rst),
    .new_addr(new_addr),
    .cur_addr(cur_addr)
);

// InstructionMemory 256*32
ROM rom(
    .a(cur_addr[9:2]),          // 读地址
    .spo(insturction)      // 读数据
);

/*
module Control(
    input [5:0] insturction,
    output reg RegDst,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg ALUOp1, ALUOp0
    output reg Jump
);
*/
Control control(
    .instruction(op),
    .RegDst(RegDst),
    .ALUSrc(ALUSrc),
    .MemtoReg(MemtoReg),
    .RegWrite(RegWrite),
    .MemRead(),
    .MemWrite(MemWrite),
    .Branch(Branch),
    .ALUOp1(Op1),
    .ALUOp0(Op0),
    .Jump(Jump)
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
    .ra0(ReadReg1),
    .rd0(ReadData1),
    .ra1(ReadReg2),
    .rd1(ReadData2),
    .wa(WriteReg),
    .we(RegWrite),
    .wd(WriteData)
);

/*
module Mux5(
    input control,
    input [4:0] in1, in0,
    output [4:0] out
);
*/
Mux5 mux0(
    .control(RegDst),
    .in0(ReadReg2),
    .in1(ReadReg3),
    .out(WriteReg)
);

Mux32 mux1(
    .control(ALUSrc),
    .in0(ReadData2),
    .in1(extendImm),
    .out(ALUin2)
);

/*
module Sign_extend(
    input [15:0] imm,
    output [31:0] extendImm
);
*/
Sign_extend signExtend(
    .imm(imm),
    .extendImm(extendImm)
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
ALU alu1(
    .y(ALUresult1),
    .zf(zero),
    .cf(),
    .of(),
    .a(Mux6out),
    .b(ALUin2),
    .m(ALUOp)
);

/*
module bus(
    input clk,
    input I_step, I_flag, // btn
    input [15 : 0] I_data,
    output [15 : 0] O_led,
    input [7 : 0] Mem_a,
    input [31 : 0] Mem_d,
    input Mem_we,
    output [31 : 0] Mem_spo
);
// DataMemory 256*32
RAM ram (
  .a(ALUresult1[9 : 2]),  // input wire [7 : 0]  地址
  .d(ReadData2),     // input wire [31 : 0] d 写数据
  .clk(clk),         // input wire clk
  .we(MemWrite),     // input wire we 写使能
  .spo(MemReadData)  // output wire [31 : 0] spo 读数据
);*/
bus BUS(
    .clk(clk),
    .I_step(I_step),
    .I_flag(I_flag),
    .I_data(I_data),
    .O_led(O_led),
    .Mem_a(ALUresult1[9 : 2]),  // input wire [7 : 0]  
    //.Mem_a(ALUresult1[7 : 0]),  // input wire [7 : 0]  地址
    .Mem_d(ReadData2),     // input wire [31 : 0] d 写数据
    .Mem_we(MemWrite),     // input wire we 写使能
    .Mem_spo(MemReadData)  // output wire [31 : 0] spo 读数据
);

Mux32 mux2(
    .control(MemtoReg),
    .in0(ALUresult1),
    .in1(MemReadData),
    .out(WriteData)
);

endmodule