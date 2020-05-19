module signal_edge(
    input clk,
    input button,
    output button_edge);
reg button_r1,button_r2;
always@(posedge clk)
    button_r1 <= button;
always@(posedge clk)
    button_r2 <= button_r1;
assign button_edge = button_r1 & (~button_r2);
endmodule

module ALU
#(parameter WIDTH = 32) 	//数据宽度
(output reg [WIDTH-1:0] y, 		//运算结果
output reg zf, 					//零标志
output reg cf, 					//进位/借位标志
output reg of, 					//溢出标志
input [WIDTH-1:0] a, b,		//两操作数
input [2:0] m		    	//操作类型
);
always@(*)
    begin
        case(m)
            3'b000: // +
            begin
                {cf, y} = a + b;
                of = (~a[WIDTH-1] & ~b[WIDTH-1] & y[WIDTH-1])
                    | (a[WIDTH-1] & b[WIDTH-1] & ~y[WIDTH-1]);
                zf = ~|y;
            end
            3'b001: // -
            begin
                {cf, y} = a - b;
                of = (~a[WIDTH-1] & b[WIDTH-1] & y[WIDTH-1])
                    | (a[WIDTH-1] & ~b[WIDTH-1] & ~y[WIDTH-1]);
                zf = ~|y;
            end
            3'b010: // &
            begin
                y = a & b;
                zf = ~|y;
                cf = 0;
                of = 0;
            end
            3'b011: // |
            begin
                y = a | b;
                zf = ~|y;
                cf = 0;
                of = 0;
            end
            3'b100: // ^
            begin
                y = a ^ b;
                zf = ~|y;
                cf = 0;
                of = 0;
            end
            default:
            begin
                y = 0;
                zf = 0;
                cf = 0;
                of = 0;
            end
        endcase
end
endmodule

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
    input [WIDTH-1:0] wd, 	    	//写端口数据
    input [4:0] ra2,				//读端口2地址
    output reg [WIDTH-1:0] rd2    	    //读端口2数据
);
reg [WIDTH-1:0] mem [255:0];
// 初始化 RAM 的内容
initial
begin
    //$readmemh("C:/Users/mi/Desktop/text.txt", mem, 0, 255);
    $readmemh("C:/Users/mi/Desktop/initReg.vec", mem, 0, 255);
end
// 异步读
always@(*)
begin
    rd0 = mem[ra0];
    rd1 = mem[ra1];
    rd2 = mem[ra2];
end
// 同步写
always@(posedge clk)
begin
    if(we)
        mem[wa] <= wd; 
end
endmodule

module Mux5(
    input control,
    input [4:0] in1, in0,
    output [4:0] out
);
assign out = control? in1:in0;
endmodule

module Mux32(
    input control,
    input [31:0] in1, in0,
    output [31:0] out
);
assign out = control? in1:in0;
endmodule

module Sign_extend(
    input [15:0] imm,
    output [31:0] extendImm
);
assign extendImm[15:0] = imm;
assign extendImm[31:16] = imm[15] ? 16'hffff : 16'h0000;
endmodule

module Control(
    input [5:0] instruction,
    output reg RegDst,
    output reg ALUSrc,
    output reg MemtoReg,
    output reg RegWrite,
    output reg MemRead,
    output reg MemWrite,
    output reg Branch,
    output reg ALUOp1, ALUOp0,
    output reg Jump
);
// add addi lw sw beq j
// x 都归为 0
always @(instruction)
begin
    case(instruction)
        6'b000000: // add
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'b1001000100;
        6'b100011: // lw
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'b0111100000;
        6'b101011: // sw
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'bx1x0010000;
        6'b000100: // beq
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'bx0x0001010;
        6'b001000: // addi
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'b0101000000;
        6'b000010: // j
            {RegDst, ALUSrc, MemtoReg, RegWrite, MemRead, 
            MemWrite, Branch, ALUOp1, ALUOp0, Jump} <= 10'bxxx000x011;
    endcase
end
endmodule

module ALUControl(
    input Op1, Op0,
    input [5 : 0] funct,
    output reg [2 : 0] ALUOp
);
always@(*)
begin
    case({Op1, Op0})
        2'b00: ALUOp <= 3'b000;
        2'b01: ALUOp <= 3'b001;
        2'b10: begin
            case(funct)
                6'b100000: ALUOp <= 3'b000;
                6'b100010: ALUOp <= 3'b001;
                6'b100100: ALUOp <= 3'b010;
                6'b100101: ALUOp <= 3'b011;
                default: ALUOp <= 3'b111;
            endcase
        end
    endcase
end
endmodule

module PC(
    input clk,
    input rst,
    input [31:0] new_addr,
    output reg [31:0] cur_addr
);
initial
    cur_addr <= 0;
always@(posedge clk or posedge rst)
begin
    if(rst)
        cur_addr <= 0;
    else
        cur_addr <= new_addr;
end
endmodule