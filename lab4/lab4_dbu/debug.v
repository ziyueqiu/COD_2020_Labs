module DBU(
    input clk,
    input rst,
    input succ,
    input step,
    input [2 : 0] sel,
    input m_rf,
    input inc,
    input dec,
    output [15 : 0] led,
    output reg [7:0] SSEG_CA, // seg
    output reg [7:0] SSEG_AN // an
);
wire run;
wire step_edge, rst_edge;
reg [8 : 0] m_rf_addr;
wire [31 : 0] m_data, rf_data, data0, data;
assign data0 = m_rf? m_data : rf_data;
assign run = succ? clk : step_edge;

// 计数
always@(posedge inc or posedge rst)
begin
    if(rst)
        m_rf_addr <= 0;
    else
        m_rf_addr <= m_rf_addr+1;
end

always@(posedge dec or posedge rst)
begin
    if(rst)
        m_rf_addr <= 0;
    else
        m_rf_addr <= m_rf_addr-1;
end

// 七段数码管
reg [31 : 0] a;
wire [63 : 0] spo;
dist_mem_gen_1 dist_mem_gen_0(
    .a (a[3:0]),
    .spo (spo[7:0])
);
dist_mem_gen_1 dist_mem_gen_1(
    .a (a[7:4]),
    .spo (spo[15:8])
);
dist_mem_gen_1 dist_mem_gen_2(
    .a (a[11:8]),
    .spo (spo[23:16])
);
dist_mem_gen_1 dist_mem_gen_3(
    .a (a[15:12]),
    .spo (spo[31:24])
);
dist_mem_gen_1 dist_mem_gen_4(
    .a (a[19:16]),
    .spo (spo[39:32])
);
dist_mem_gen_1 dist_mem_gen_5(
    .a (a[23:20]),
    .spo (spo[47:40])
);
dist_mem_gen_1 dist_mem_gen_6(
    .a (a[27:24]),
    .spo (spo[55:48])
);
dist_mem_gen_1 dist_mem_gen_7(
    .a (a[31:28]),
    .spo (spo[63:56])
);

// 分时复用
reg [18:0] timer;
//always@(posedge clk or posedge rst)
always@(posedge clk)
begin
    if(rst)
        timer <= 0;
    else
        timer <= timer+1;
    case (timer[18:16])
        3'b000: begin
            SSEG_CA <= spo[7:0];
            SSEG_AN <= 8'b11111110;
        end
        3'b001: begin
            SSEG_CA <= spo[15:8];
            SSEG_AN <= 8'b11111101;
        end
        3'b010: begin
            SSEG_CA <= spo[23:16];
            SSEG_AN <= 8'b11111011;
        end
        3'b011: begin
            SSEG_CA <= spo[31:24];
            SSEG_AN <= 8'b11110111;
        end
        3'b100: begin
            SSEG_CA <= spo[39:32];
            SSEG_AN <= 8'b11101111;
        end
        3'b101: begin
            SSEG_CA <= spo[47:40];
            SSEG_AN <= 8'b11011111;
        end
        3'b110: begin
            SSEG_CA <= spo[55:48];
            SSEG_AN <= 8'b10111111;
        end
        3'b111: begin
            SSEG_CA <= spo[63:56];
            SSEG_AN <= 8'b01111111;
        end
        default: SSEG_AN <= 8'b11111111;
    endcase
end

always@(*)
begin
    if(sel == 0)
        a <= data0;
    else
        a <= data;
end

/*
module signal_edge(
    input clk,
    input button,
    output button_edge);*/
signal_edge step0(
    .clk(clk),
    .button(step),
    .button_edge(step_edge)
);

signal_edge rst0(
    .clk(clk),
    .button(rst),
    .button_edge(rst_edge)
);


cpu cpu(
    .clk(run),
    .rst(rst_edge),
    .m_data(m_data),
    .rf_data(rf_data),
    .data(data),
    .sel(sel),
    .led(led),
    .m_rf_addr(m_rf_addr)
);

endmodule