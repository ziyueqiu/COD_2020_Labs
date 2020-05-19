module sort
#(parameter N = 4) 			//数据宽度
(output reg [N-1:0] s0 = 0, s1 = 0, s2 = 0, s3 = 0, 	//排序后的四个数据（递增）
output reg done, 				//排序结束标志
input [N-1:0] x0, x1, x2, x3,	//原始输入数据
input clk, rst				//时钟（上升沿有效）、复位（高电平有效）
);

// 状态数
parameter LOAD = 3'b000;
parameter CX01 = 3'b001;
parameter CX12 = 3'b010;
parameter CX23 = 3'b011;
parameter CX01A = 3'b100;
parameter CX12A = 3'b101;
parameter CX01B = 3'b110;
parameter HLT = 3'b111;
parameter SUB = 3'b001;

// 变量定义
wire [N-1:0] i0, i1, i2, i3; // in
wire [N-1:0] r0, r1, r2, r3; // register
wire [N-1:0] a, b, y; // ALU 两个输入 和 输出
wire of;
reg en0, en1, en2, en3; // enable
reg m0, m1, m2, m3, m4, m5; // mux
reg [2:0] current_state, next_state;

// Data Path
register #(N) 
    R0 (.din(i0), .en(en0), .rst(rst), .clk(clk), .dout(r0)), 
    R1 (.din(i1), .en(en1), .rst(rst), .clk(clk), .dout(r1)), 
    R2 (.din(i2), .en(en2), .rst(rst), .clk(clk), .dout(r2)),
    R3 (.din(i3), .en(en3), .rst(rst), .clk(clk), .dout(r3));
alu #(N) ALU (.a(a), .b(b), .m(SUB), .zf(), .cf(), .of(of), .y(y));
mux #(N)
    M0 (.sel(m0), .a(r0), .b(r2), .out(a)), 
    M1 (.sel(m1), .a(x0), .b(r1), .out(i0)),
    M2 (.sel(m2), .a(a), .b(x1), .out(i1)),
	M3 (.sel(m3), .a(b), .b(x2), .out(i2)),
    M4 (.sel(m4), .a(r2), .b(x3), .out(i3)),
    M5 (.sel(m5), .a(r1), .b(x3), .out(b));

// Control Unit
always @(posedge clk, posedge rst)
	if (rst)
	begin
	   current_state <= LOAD;
	   done <= 0;
	end
	else
	   current_state <= next_state;

always @(*) begin
   case (current_state)
      LOAD: next_state = CX01;
      CX01: next_state = CX12;
      CX12: next_state = CX23;
      CX23: next_state = CX01A;
      CX01A: next_state = CX12A;
      CX12A: next_state = CX01B;
      CX01B: next_state = HLT;
      HLT: next_state = HLT;
      default: next_state = HLT;
   endcase
end

// 两段式：控制信号--组合输出
// 
always @(*) begin
    {m0, m1, m2, m3, m4, m5, en0, en1, en2, en3, done} = 10'h0;
    case (current_state)
        LOAD: {m2, m3, m4, en0, en1, en2, en3} = 7'b1111111;
        CX01, CX01A, CX01B:
        begin
            m1 = 1; en0 = ~(of^y[N-1]); en1 = ~(of^y[N-1]);
            {m0, m2, m5, en2, en3} = 5'b0;
        end
        CX12, CX12A:
        begin
            m0 = 1; en1 = (of^y[N-1]); en2 = (of^y[N-1]);
            {m2, m3, m4, en0, en3} = 5'b0;
        end
        CX23:
        begin
            m0 = 1; m5 = 1; en2 = ~(of^y[N-1]); en3 = ~(of^y[N-1]);
            {m3, m4, en0, en1} = 4'b0;
        end
        HLT:
        begin
            {en0, en1, en2, en3} = 4'b0;
            //s0 = r0; s1 = r1; s2 = r2; s3 = r3;
            done = 1;
        end
    endcase
end
always @(posedge clk)
begin
    if(done == 1)
    begin
        s0 <= r0;
        s1 <= r1;
        s2 <= r2;
        s3 <= r3;
    end
end
endmodule