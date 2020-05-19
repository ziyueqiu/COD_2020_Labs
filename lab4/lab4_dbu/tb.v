module tb();
reg clk, rst, succ, step;
reg [2:0] sel;
reg m_rf;
reg inc, dec;
wire [15 : 0] led;
wire [7 : 0] SSEG_CA, SSEG_AN;

DBU dbu(
    .clk(clk),
    .rst(rst),
    .succ(succ),
    .step(step),
    .sel(sel),
    .m_rf(m_rf),
    .inc(inc),
    .dec(dec),
    .led(led),
    .SSEG_CA(SSEG_CA),
    .SSEG_AN(SSEG_AN)
);

parameter PERIOD = 10;
/*
initial
begin
    rst = 0;
    succ = 1;
    inc = 0;
    dec = 0;
    # PERIOD;
    rst = 1;
    sel = 0;
    m_rf = 1;
    # PERIOD;
    rst = 0;
end

initial
begin
    inc = 0;
    # (PERIOD*2);
    inc = 1;
    # PERIOD;
    inc = 0;
end
*/
initial
begin
    rst = 0;
    succ = 1;
    # PERIOD;
    rst = 1;
    sel = 7;
    # PERIOD;
    rst = 0;
end

initial
begin
    clk = 0;
    repeat (65) // 待定
        #(PERIOD/2) clk = ~clk;
    $finish;
end
endmodule