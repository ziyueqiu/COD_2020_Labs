module fifo_tb();
reg clk, rst;
reg [7:0] din;
reg en_in, en_out;
wire [7:0] dout;
wire [4:0] count;

fifo F(.clk(clk), .rst(rst), .din(din), .en_in(en_in), .en_out(en_out), .dout(dout), .count(count));

parameter PERIOD = 10;

initial
begin
    clk = 0;
    repeat (300) // 待定
        #(PERIOD/2) clk = ~clk;
    $finish;
end

initial
begin
    rst = 1;
    #PERIOD rst = 0;
end

initial
begin
    en_in = 0;
    en_out = 0;
    # (PERIOD*3);

    en_in = 1;
    en_out = 0;
    din = 60;
    # (PERIOD*10);

    en_in = 0;
    en_out = 0;
    # (PERIOD*2);

    en_in = 1;
    en_out = 0;
    din = 2;
    # (PERIOD*10);

    en_out = 1;
    en_in = 0;
    din = 8;
    # (PERIOD*10);

    en_in = 0;
    en_out = 0;
    # (PERIOD*2);
    
    en_out = 1;
    en_in = 0;
    din = 8;
    # (PERIOD*10);

    en_in = 0;
    en_out = 0;
    # (PERIOD*2);

    en_out = 1;
    en_in = 0;
    din = 100;
    # (PERIOD*10);
    
    en_in = 0;
    en_out = 0;
    # (PERIOD*2);
    
    en_in = 1;
    en_out = 0;
    din = 3;
    # (PERIOD*10);
    
    en_in = 0;
    en_out = 0;
    # (PERIOD*2);

    en_out = 1;
    en_in = 0;
    din = 100;
    # (PERIOD*10);
end
endmodule