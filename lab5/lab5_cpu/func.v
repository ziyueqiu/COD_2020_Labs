// mux
module Mux2_5(
    input control,
    input [4:0] in1, in0,
    output [4:0] out
);
assign out = control? in1:in0;
endmodule

module Mux2_32(
    input control,
    input [31:0] in1, in0,
    output [31:0] out
);
assign out = control? in1:in0;
endmodule

module Mux2_9(
    input control,
    input [8:0] in1, in0,
    output [8:0] out
);
assign out = control? in1:in0;
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
    input stall,
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
        if(~stall)
            cur_addr <= new_addr;
        else
            cur_addr <= cur_addr;
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
    input [WIDTH-1:0] wd 	    	//写端口数据
);
reg [WIDTH-1:0] mem [255:0];
// 初始化 RAM 的内容
initial
begin
    //$readmemh("C:/Users/mi/Desktop/text.txt", mem, 0, 255);
    $readmemh("C:/Users/mi/Desktop/lab5_cpu/initReg.vec", mem, 0, 255);
end
// 异步读
always@(*)
begin
    rd0 = mem[ra0];
    rd1 = mem[ra1];
end
// 同步写
always@(negedge clk)
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
    input [5:0] instruction,
    output reg EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
    output reg M_Branch, M_MemRead, M_MemWrite,
    output reg WB_RegWrite, WB_MemtoReg
);
// add addi lw sw beq j
// x 都归为 0
always @(instruction)
begin
    case(instruction)
        6'b000000: // add
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b110000010;
        6'b100011: // lw
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b000101011;
        6'b101011: // sw
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b00010010x;
        6'b000100: // beq
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b00101000x;
        6'b001000: // addi
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b000100010;
        /*6'b000010: // j
            {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b;*/
        default:{EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc, 
            M_Branch, M_MemRead, M_MemWrite,
            WB_RegWrite, WB_MemtoReg} <= 9'b0;
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

module ForwardingUnit(
    input [4 : 0] ID_EX_Rt, ID_EX_Rs,
    input [4 : 0] EX_MEM_Rd, 
    input [4 : 0] MEM_WB_Rd, 
    input EX_MEM_RegWrite, MEM_WB_RegWrite,
    input EX_MEM_MemWrite, // 加
    output reg [1 : 0] ForwardA, ForwardB, 
    output reg ForwardC // 加
);
always@(*)
begin
    if(EX_MEM_RegWrite & (EX_MEM_Rd!=0) & (EX_MEM_Rd == ID_EX_Rs))
        ForwardA <= 2'b10;
    else if(MEM_WB_RegWrite & (MEM_WB_Rd!=0) 
            //& ~(EX_MEM_RegWrite & (EX_MEM_Rd!=0)
            //& (EX_MEM_Rd!=ID_EX_Rs))
            & (MEM_WB_Rd == ID_EX_Rs))
        ForwardA <= 2'b01;
    else
        ForwardA <= 2'b00;

    if(EX_MEM_RegWrite & (EX_MEM_Rd!=0) & (EX_MEM_Rd == ID_EX_Rt))
        ForwardB <= 2'b10;
    else if(MEM_WB_RegWrite & (MEM_WB_Rd!=0) 
            //& ~(EX_MEM_RegWrite & (EX_MEM_Rd!=0) & (EX_MEM_Rd!=ID_EX_Rt))
            & (MEM_WB_Rd == ID_EX_Rt))
        ForwardB <= 2'b01;
    else
        ForwardB <= 2'b00;
    if(EX_MEM_MemWrite & MEM_WB_RegWrite & (EX_MEM_Rd == MEM_WB_Rd))
        ForwardC <= 1;
    else
        ForwardC <= 0;
end
endmodule

module HazardDetectionUnit(
    input EX_MemRead, EX_Branch, EX_zero,
    input [4 : 0] ID_Rs, ID_Rt, 
    input [4 : 0] EX_Rtd, 
    input jump,
    input rst,
    output reg stall, Clrn1, Clrn2
);
always@(*)
begin
    if(rst == 1)
    begin
        stall <= 1;
        Clrn1 <= 0;
        Clrn2 <= 0;
    end
    else if(EX_MemRead & 
        ((EX_Rtd == ID_Rs) | (EX_Rtd == ID_Rt)) &
        (EX_Rtd != 0)) // lw 气泡
    begin
        stall <= 1;
        Clrn1 <= 1;
        Clrn2 <= 0;
    end
    else if(EX_Branch & EX_zero) // beq
    begin
        stall <= 0;
        Clrn1 <= 0;
        Clrn2 <= 0;
    end
    else if(jump) // jump
    begin
        stall <= 0;
        Clrn1 <= 0;
        Clrn2 <= 1;
    end
    else
    begin
        stall <= 0;
        Clrn1 <= 1;
        Clrn2 <= 1;
    end
end
endmodule

module IF_ID_Registers(
    input clk,
    input [31 : 0] IF_PCadd,
    input [31 : 0] IF_instruction, 
    input Clrn, 
    input stall,
    output reg [31 : 0] ID_PCadd,
    output reg [31 : 0] ID_instruction
);
always@(posedge clk)
begin
    if(~Clrn)
    begin
        ID_PCadd <= 0;
        ID_instruction <= 32'hffffffff;
    end
    else if(stall)
    begin
        ID_PCadd <= ID_PCadd;
        ID_instruction <= ID_instruction;
    end
    else
    begin
        ID_PCadd <= IF_PCadd;
        ID_instruction <= IF_instruction;
    end
end
endmodule

module ID_EX_Registers(
    input clk,
    input Clrn,
    input [8 : 0] ID_Signal, 
    input [31 : 0] ID_RegData0, ID_RegData1,
    input [31 : 0] extendImm, ID_PCadd,
    input [4 : 0] ID_Rs, ID_Rt, ID_Rd,
    output reg EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc,
            EX_Branch, EX_MemRead, EX_MemWrite,
            EX_RegWrite, EX_MemtoReg,
    output reg [31 : 0] EX_RegData0, EX_RegData1, 
    output reg [31 : 0] EX_extendImm, EX_PCadd, 
    output reg [4 : 0] EX_Rs, EX_Rt, EX_Rd
);
always@(posedge clk)
begin
    if(~Clrn)
    begin
        {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc,
        EX_Branch, EX_MemRead, EX_MemWrite,
        EX_RegWrite, EX_MemtoReg} <= 9'b0;
        EX_RegData1 <= 32'b0;
        EX_RegData0 <= 32'b0;
        EX_extendImm <= 32'b0;
        EX_PCadd <= 32'b0;
        EX_Rs <= 5'b0;
        EX_Rt <= 5'b0;
        EX_Rd <= 5'b0;
    end
    else
    begin
        {EX_RegDst, EX_ALUOp1, EX_ALUOp0, EX_ALUSrc,
        EX_Branch, EX_MemRead, EX_MemWrite,
        EX_RegWrite, EX_MemtoReg} <= ID_Signal;
        EX_RegData1 <= ID_RegData1;
        EX_RegData0 <= ID_RegData0;
        EX_extendImm <= extendImm;
        EX_PCadd <= ID_PCadd;
        EX_Rs <= ID_Rs;
        EX_Rt <= ID_Rt;
        EX_Rd <= ID_Rd;
    end
end
endmodule

module EX_MEM_Registers(
    input clk,
    input EX_Branch, EX_MemRead, EX_MemWrite,
        EX_RegWrite, EX_MemtoReg,
    //input zero,
    input [31 : 0] ALUy, EX_RegData1,
    input [4 : 0] EX_Rtd,
    output reg MEM_Branch, MEM_MemRead, MEM_MemWrite,
        MEM_RegWrite, MEM_MemtoReg,
    //output reg MEM_zero,
    output reg [31 : 0] MEM_ALUy, MEM_RegData1,
    output reg [4 : 0] MEM_Rd
);
always@(posedge clk)
begin
    {MEM_Branch, MEM_MemRead, MEM_MemWrite,
    MEM_RegWrite, MEM_MemtoReg} <= 
    {EX_Branch, EX_MemRead, EX_MemWrite,
    EX_RegWrite, EX_MemtoReg};
    //MEM_zero <= zero;
    MEM_ALUy <= ALUy;
    MEM_RegData1 <= EX_RegData1;
    MEM_Rd <= EX_Rtd;
end
endmodule


module MEM_WB_Registers(
    input clk,
    input MEM_RegWrite, MEM_MemtoReg,
    input [31 : 0] MEM_Data,
    input [31 : 0] MEM_ALUy,
    input [4 : 0] MEM_Rd,
    output reg WB_RegWrite, WB_MemtoReg,
    output reg [31 : 0] WB_ALUy, WB_Data,
    output reg [4 : 0] WB_Rd
);
always@(posedge clk)
begin
    {WB_RegWrite, WB_MemtoReg} <= 
        {MEM_RegWrite, MEM_MemtoReg};
    WB_Data <= MEM_Data;
    WB_ALUy <= MEM_ALUy;
    WB_Rd <= MEM_Rd;
end
endmodule