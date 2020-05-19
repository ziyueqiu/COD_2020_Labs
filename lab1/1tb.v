module tb();

reg [3:0] a, b;		//两操作数
reg [2:0] m;		//操作类型

alu alu(
.a(a),
.b(b),
.m(m),
.y(),
.zf(),
.cf(),
.of()
);
initial
begin
    repeat(40)
    begin
        a = $random % 16;
        b = $random % 16;
        m = $random % 8;
        #20;
    end
    $stop;
end
endmodule