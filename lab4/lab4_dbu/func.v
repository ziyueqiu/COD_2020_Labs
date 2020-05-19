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

// mux
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

module Mux4_32(
    input [1:0] control,
    input [31:0] in11, in10, in01, in00,
    output [31:0] out
);
assign out = control[1]? (control[0]?in11:in10):(control[0]?in01:in00);
endmodule

module Mux3_32(
    input [1:0] control,
    input [31:0] in10, in01, in00,
    output [31:0] out
);
assign out = control[1]? (control[0]?32'b0:in10):(control[0]?in01:in00);
endmodule


module PC(
    input clk,
    input rst,
    input PCwe,
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
        if(PCwe)
            cur_addr <= new_addr;
        else
            cur_addr <= cur_addr;
end
endmodule

module Instruction(
    input [31 : 0] MemData,
    input clk,
    input IRWrite,
    output reg [31 : 0] instruction
);
initial
    instruction <= 0;
always@(posedge clk)
    if(IRWrite)
        instruction <= MemData;
    else
        instruction <= instruction;
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
    $readmemh("C:/Users/mi/Desktop/lab4_cpu/initReg.vec", mem, 0, 255);
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
        if(wa != 5'b0)
            mem[wa] <= wd; 
end
endmodule

module Sign_extend(
    input [15:0] imm,
    output [31:0] extendImm
);
assign extendImm[15:0] = imm;
assign extendImm[31:16] = imm[15] ? 16'hffff : 16'h0000;
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

module Control(
    input [5 : 0] Op,
    input clk,
    input rst,
    output reg PCWriteCond,
    output reg [1 : 0] PCSource,
    output reg PCWrite,
    output reg [1 : 0] ALUOp,
    output reg lorD,
    output reg [1 : 0] ALUSrcB,
    output reg ALUSrcA,
    output reg MemWrite, RegWrite, MemRead,
    output reg MemtoReg,
    output reg IRWrite, RegDst
);
/*
{PCWriteCond, PCSource, PCWrite, ALUOp, lorD, ALUSrcB, 
ALUSrcA, MemWrite, RegWrite, MemRead, MemtoReg, IRWrite, RegDst}*/
localparam S0 = 4'd1;
localparam S1 = 4'd2;
localparam S2 = 4'd3;
localparam S3 = 4'd4;
localparam S4 = 4'd5;
localparam S5 = 4'd6;
localparam S6 = 4'd7;
localparam S7 = 4'd8;
localparam S8 = 4'd9;
localparam S9 = 4'd10;
localparam S10 = 4'd11;
localparam S11 = 4'd12;
localparam Start = 4'd0;

reg [3 : 0] curr_state = Start, next_state;

always@(posedge clk or posedge rst)
    if(rst)
        curr_state <= Start;
    else
        curr_state <= next_state;

always@(*)
    case(curr_state)
        Start: next_state = S0;
        S0: next_state = S1;
        S1: 
            case(Op)
                6'b000000: // add
                    next_state = S6;
                6'b001000: // addi
                    next_state = S10;
                6'b100011, 6'b101011: // lw sw
                    next_state = S2;
                6'b000100: // beq
                    next_state = S8;
                6'b000010: // j
                    next_state = S9;
                default: ;
            endcase
        S2: 
            if(Op == 6'b100011) // lw
                next_state = S3;
            else if(Op == 6'b101011) // sw
                next_state = S5;
            else
                next_state = curr_state;
        S3: next_state = S4;
        S4: next_state = S0;
        S5: next_state = S0;
        S6: next_state = S7;
        S7, S8, S9, S11: next_state = S0;
        S10: next_state = S11;
        default: next_state = curr_state;
    endcase

always@(posedge clk or posedge rst)
    if(rst)
        {PCWriteCond, PCSource, PCWrite, ALUOp, lorD, ALUSrcB, 
        ALUSrcA, MemWrite, RegWrite, MemRead, MemtoReg, IRWrite, RegDst}<=16'b0;
    else
    case(next_state)
        S0:begin
            MemRead <= 1;
            ALUSrcA <= 0;
            lorD <= 0;
            IRWrite <= 1;
            ALUSrcB <= 2'b01;
            ALUOp <= 2'b00;
            PCSource <= 2'b00;
            PCWrite <= 1;
            {PCWriteCond, MemWrite, RegWrite, MemtoReg, RegDst} <= 5'b0;
        end
        S1:begin
            ALUSrcA <= 0;
            ALUSrcB <= 2'b11;
            ALUOp <= 2'b00;
            {PCWriteCond, PCSource, PCWrite, lorD, MemWrite,
            RegWrite, MemRead, MemtoReg, IRWrite, RegDst} <= 11'b0;
        end
        S2:begin
            ALUSrcA <= 1;
            ALUSrcB <= 2'b10;
            ALUOp <= 2'b00;
            {PCWriteCond, PCSource, PCWrite, lorD, MemWrite,
            RegWrite, MemRead, MemtoReg, IRWrite, RegDst} <= 11'b0;
        end
        S3:begin
            MemRead <= 1;
            lorD <= 1;
            {PCWriteCond, PCSource, PCWrite, ALUOp, ALUSrcB, ALUSrcA, 
            MemWrite, RegWrite, MemtoReg, IRWrite, RegDst} <= 14'b0;
        end
        S4:begin
            RegDst <= 0;
            RegWrite <= 1;
            MemtoReg <= 1;
            {PCWriteCond, PCSource, PCWrite, ALUOp, lorD, ALUSrcB, 
            ALUSrcA, MemWrite, MemRead, IRWrite} <= 13'b0;
        end
        S5:begin
            MemWrite <= 1;
            lorD <= 1;
            {PCWriteCond, PCSource, PCWrite, ALUOp, ALUSrcB, ALUSrcA, 
            MemRead, RegWrite, MemtoReg, IRWrite, RegDst} <= 14'b0;
        end
        S6:begin
            ALUSrcA <= 1;
            ALUSrcB <= 2'b00;
            ALUOp <= 2'b10;
            {PCWriteCond, PCSource, PCWrite, lorD, MemWrite,
            RegWrite, MemRead, MemtoReg, IRWrite, RegDst} <= 11'b0;
        end
        S7:begin
            RegDst <= 1;
            RegWrite <= 1;
            MemtoReg <= 0;
            {PCWriteCond, PCSource, PCWrite, ALUOp, lorD, ALUSrcB, 
            ALUSrcA, MemWrite, MemRead, IRWrite} <= 13'b0;
        end
        S8:begin
            ALUSrcA <= 1;
            ALUSrcB <= 2'b00;
            ALUOp <= 2'b01;
            PCWriteCond <= 1;
            PCSource <= 2'b01;
            {PCWrite, lorD, MemWrite,
            RegWrite, MemRead, MemtoReg, IRWrite, RegDst} <= 8'b0;
        end
        S9:begin
            PCWrite <= 1;
            PCSource <= 2'b10;
            {PCWriteCond, ALUOp, lorD, ALUSrcB, ALUSrcA, MemWrite, 
            RegWrite, MemRead, MemtoReg, IRWrite, RegDst} <= 13'b0;
        end
        S10:begin
            {PCWriteCond, PCWrite, lorD, MemWrite, MemtoReg,
            IRWrite, ALUSrcA, RegWrite, RegDst, ALUSrcB[1:0],
            PCSource[1:0], ALUOp[1:0]} <= 15'b0000x_010x10_0000;
        end
        S11:begin
            {PCWriteCond, PCWrite, lorD, MemWrite, MemtoReg,
            IRWrite, ALUSrcA, RegWrite, RegDst, ALUSrcB[1:0],
            PCSource[1:0], ALUOp[1:0]} <= 15'b00000_001000_0000;
        end
        default:;
    endcase
endmodule

module Register(
    input clk,
    input [31 : 0] in,
    output reg [31 : 0] out
);
always@(posedge clk)
    out <= in;
endmodule