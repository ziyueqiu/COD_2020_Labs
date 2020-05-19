module register
#(parameter N = 4)
(input [N-1:0] din,
input en, rst, clk,
output reg [N-1:0] dout
);
always@(posedge clk or posedge rst)
begin
if(rst)
    dout <= 0;
else if(en == 1)
    dout <= din;
end
endmodule

module mux
#(parameter N = 4)
(input sel,
input [N-1:0] a,b,
output reg [N-1:0] out
);
always @(a,b,sel)
begin
    if(sel == 1'b0)
        out <= a;
    else
        out <= b;
end
endmodule