module tb();

reg [4:0] a;
reg [15:0] wd;
reg clk;
reg we, en;
wire [15:0] rdD, rdB;

IP test(
    .a(a),
    .wd(wd),
    .clk(clk),
    .we(we),
    .en(en),
    .rdD(rdD),
    .rdB(rdB)
);

parameter PERIOD = 10;

initial
begin
    clk = 0;
    repeat (8) // 待定
        #(PERIOD/2) clk = ~clk;
    $finish;
end

initial
begin
    a = 1;
    wd = 2;
    we = 1;
    en = 1;

    # (PERIOD);
    a = 3;
    wd = 4;
    we = 0;
    en = 1;
    
    # (PERIOD);
    a = 5;
    wd = 7;
    we = 1;
    en = 1;

end
endmodule