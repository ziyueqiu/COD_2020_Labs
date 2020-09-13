/*
btn1 I_step高电平 存 flag_data 进 flag_place
btn2 I_flag高电平 读入SW 存16位SW进 input_addr
*/
module bus(
    input clk,
    input I_step, I_flag, // btn
    input [15 : 0] I_data,
    output [15 : 0] O_led,
    input [7 : 0] Mem_a,
    input [31 : 0] Mem_d,
    input Mem_we,
    output reg [31 : 0] Mem_spo
);
localparam flag_place = 8'hfe;
localparam input_addr = 8'hff;
reg [31 : 0] flag_data=0;
reg [31 : 0] input_data=0;
wire [31 : 0] inter_spo;

assign O_led = input_data[15 : 0];

always@(posedge clk)
begin
    if(I_step)
        flag_data <= 32'b1; // 特殊值
    else
        flag_data <= 32'b0;
end

always@(*) // MEM 读
begin
    if(Mem_a == flag_place)
        Mem_spo = flag_data;
    else if(Mem_a == input_addr)
        Mem_spo = input_data;
    else
        Mem_spo = inter_spo;
end

always@(posedge clk) // MEM 写
begin
    if(Mem_we)
    begin
        if(Mem_a == input_addr)
            input_data <= Mem_d;
        else
            input_data <= input_data;
    end
    else
    begin
        if(I_flag)
            input_data <= {16'b0, I_data};
        else
            input_data <= input_data;
    end
end

// DataMemory 256*32
RAM ram (
  .a(Mem_a),  // input wire [7 : 0]  地址
  .d(Mem_d),     // input wire [31 : 0] d 写数据
  .clk(clk),         // input wire clk
  .we(Mem_we),     // input wire we 写使能
  .spo(inter_spo)  // output wire [31 : 0] spo 读数据
);

endmodule