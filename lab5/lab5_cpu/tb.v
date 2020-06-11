module tb();
reg clk;
reg rst;

CPU cpu(
    .clk(clk),
    .rst(rst)
);
parameter PERIOD = 10;
initial
begin
    rst = 1;
    # PERIOD;
    rst = 0;
end

initial
begin
    clk = 0;
    repeat (200) // 待定
        #(PERIOD/2) clk = ~clk;
    $finish;
end
endmodule