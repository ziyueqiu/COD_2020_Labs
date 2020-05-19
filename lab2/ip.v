module IP(
input [4:0] a,
input [15:0] wd,
input clk,
input we,en,
output [15:0] rdD, rdB
);
    
dist_mem_gen_0 dist0 (
  .a(a),      // input wire [4 : 0]  地址
  .d(wd),      // input wire [15 : 0] d 写数据
  .clk(clk),  // input wire clk
  .we(we),    // input wire we 写使能
  .spo(rdD)  // output wire [15 : 0] spo 读数据
);

blk_mem_gen_0 dist1 (
  .clka(clk),    // input wire clka
  .ena(en),      // input wire ena 总使能
  .wea(we),      // input wire [0 : 0] wea 写使能
  .addra(a),  // input wire [4 : 0] addra 地址
  .dina(wd),    // input wire [15 : 0] dina 写数据
  .douta(rdB)  // output wire [15 : 0] douta 读数据
);

endmodule
