/*module signal_edge(
    input clk,
    input button,
    output button_edge);
reg button_r1,button_r2;
always@(posedge clk)
    button_r1 <= button;
always@(posedge clk)
    button_r2 <= button_r1;
assign button_redge = button_r1 & (~button_r2);
endmodule*/

module register_file				//32 x WIDTH寄存器堆
#(parameter WIDTH = 4) 	        //数据宽度
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

reg [WIDTH-1:0] mem [31:0];
// 初始化 RAM 的内容
initial
begin
    //$readmemh("C:/Users/mi/Desktop/text.txt", mem, 0, 31);
    $readmemb("C:/Users/mi/Desktop/init.vec", mem, 0, 31);
end

// 异步读
always@(*)
begin
    rd0 = mem[ra0];
    rd1 = mem[ra1];
end

// 取信号边沿
//wire we_edge;
//signal_edge CLK(.clk(clk), .button(we), .button_edge(we_edge));

// 同步写
always@(posedge clk)
begin
    if(we)
        mem[wa] <= wd; 
end
endmodule