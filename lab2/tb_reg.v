module tb();
reg clk, we;
reg [4:0] ra0, ra1, wa;
wire [3:0] rd0, rd1;
reg [3:0] wd;

register_file #(4) Register 
    (.clk(clk), .ra0(ra0), .ra1(ra1), 
    .rd0(rd0), .rd1(rd1), .wa(wa), .we(we), .wd(wd));

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

    ra0 = 1;
    ra1 = 2;
    we = 1;
    wa = 5;
    wd = 5;

    # (PERIOD);
    ra0 = 8;
    ra1 = 9;
    we = 1;
    wa = 9;
    wd = 6;

    # (PERIOD);
    ra0 = 3;
    ra1 = 4;
    we = 1;
    wa = 1;
    wd = 6;

    # (PERIOD);
    ra0 = 1;
    ra1 = 5;
    we = 1;
    wa = 1;
    wd = 4;
end
endmodule