module tb();
reg clk;
reg rst;
reg I_step, I_flag;
reg [15 : 0] I_data;
wire [15 : 0] O_led;

/*
module cpu_one_cycle( // 单周期 CPU
    input clk,        // 时钟（上升沿有效）
    input rst,        // 异步复位，高电平有效
    input I_step, I_flag, // btn
    input [15 : 0] I_data,
    output [15 : 0] O_led,
);
*/
cpu_one_cycle cpu(
    .clk(clk),
    .rst(rst),
    .I_step(I_step),
    .I_flag(I_flag),
    .I_data(I_data),
    .O_led(O_led)
);
parameter PERIOD = 10;
initial
begin
    rst = 1;
    # (PERIOD*1);
    rst = 0;
end

initial
begin
    clk = 0;
    repeat (400) // 待定
        #(PERIOD/2) clk = ~clk;
    $finish;
end

/*
btn1 I_step高电平 存 flag_data 进 flag_place
btn2 I_flag高电平 读入SW 存16位SW进 input_addr
*/
initial
begin
    I_data = 16'h9abc; // test
    I_flag = 0;
    # (PERIOD*2);
    I_flag = 1; // 改变了input_data
    I_step = 0;
    # (PERIOD);
    I_flag = 0;
    # (PERIOD*5);
    repeat(10)
    begin
        I_step = 1;
        # (PERIOD*6);
        I_step = 0;
        # (PERIOD*10);  
    end
end
endmodule