/*  CF 进位/借位标志
    加法或减法时，如果最高位产生进位或借位时置1，否则清零
    OF 溢出标志
    有符号数运算结果溢出时置1，否则清零
    ZF 零标志
    结果为零时置1，否则清零
 */

module alu
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