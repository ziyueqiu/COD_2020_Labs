module signal_edge(
    input y,rst,clk,
    output p
);

    localparam S0 = 0;
    localparam S1 = 1;
    localparam S2 = 2;

    reg [1:0]state,next_state;
    //output logic
    assign  p = (state==S1); 
    //state logic
    always @(posedge clk, posedge rst)
        if (rst) state <= S0;
        else state <= next_state; 
    //next state logic
    always @(*) 
    begin 
        next_state = state;
        case (state)
            S0: if (y) next_state = S1;
            S1:begin if (y) next_state = S2;
                     else next_state = S0;
                end
            S2: if (!y) next_state = S0;
            default: next_state = S0;
        endcase
    end
endmodule

module fifo
(
    input clk, rst,		    //时钟（上升沿有效）、异步复位（高电平有效）
    input [7:0] din,		//入队列数据
    input en_in, 		    //入队列使能，高电平有效
    input en_out,		    //出队列使能，高电平有效
    output reg [7:0] dout, 	//出队列数据
    output reg [4:0] count	//队列数据计数
);  
    reg [1:0] state,next_state;
    reg [3:0] add_r, add_w, add; //读地址，写地址，最终用到地址
    wire p1,p2;
    reg en_r,en_w; //读使能和写使能
    wire [7:0]out;

    localparam NULL = 0;
    localparam MIDDLE = 1;
    localparam FULL = 2;
    
    //DATA Path
    signal_edge EDG1(.y(en_in), .rst(rst), .clk(clk), .p(p1));
    signal_edge EDG2(.y(en_out), .rst(rst), .clk(clk), .p(p2));
    dist_mem_gen_0 FIFO(.a(add), .d(din), .clk(clk), .we(en_w), .spo(out));

    //Control Unit
    always @(posedge clk , posedge rst)
    begin
        if(rst) state <= NULL;
        else state <= next_state;
    end
    always @(*)
    begin
        if(rst)
            {count, add_r, add_w} = 0;
        en_r=0;
        en_w=0;
        next_state = state;
        case(state)
            NULL:
                if(p1)
                begin
                    en_w = 1;
                    next_state = MIDDLE;
                    add = add_w;
                    add_w = (add_w == 31)?0:add_w + 1;
                    count = count + 1;
                end 
            MIDDLE:
                begin
                    if(p1) 
                    begin
                        en_w = 1;
                        add = add_w;
                        add_w = (add_w == 31)?0 : add_w + 1;
                        count = count + 1;
                        if(count == 16) next_state = FULL;
                    end
                    else if(p2)
                    begin
                        en_r = 1;
                        add = add_r;
                        add_r = (add_r == 31)?0 : add_r + 1;
                        count = count - 1;
                        if(count == 0) next_state = NULL;
                    end    
                end
            FULL:
                if(p2)
                begin
                    en_r = 1;
                    next_state = MIDDLE;
                    add = add_r;
                    add_r = add_r + 1;
                    count = count - 1;
                end
            default: next_state = NULL;
        endcase
    end
    always @(*)
    begin
        if(rst) dout = 0;
        else if(en_r) dout = out;
    end
endmodule