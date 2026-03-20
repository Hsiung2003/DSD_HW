module register_file(
    Clk  ,
    WEN  ,
    RW   ,
    busW ,
    RX   ,
    RY   ,
    busX ,
    busY
);
input        Clk, WEN;
input  [2:0] RW, RX, RY;
input  [7:0] busW;
output [7:0] busX, busY;

reg [7:0] r0_w, r1_w, r2_w, r3_w, r4_w, r5_w, r6_w, r7_w;
reg [7:0] busX_r, busY_r;

assign busX = busX_r;
assign busY = busY_r;

initial begin
    r0_w = 8'b0000_0000;
    r1_w = 8'b0000_0000;
    r2_w = 8'b0000_0000;
    r3_w = 8'b0000_0000;
    r4_w = 8'b0000_0000;
    r5_w = 8'b0000_0000;
    r6_w = 8'b0000_0000;
    r7_w = 8'b0000_0000;
end

always @(*) begin
    case (RX)
        3'd0: busX_r = 8'b0000_0000;
        3'd1: busX_r = r1_w;
        3'd2: busX_r = r2_w;
        3'd3: busX_r = r3_w;
        3'd4: busX_r = r4_w;
        3'd5: busX_r = r5_w;
        3'd6: busX_r = r6_w;
        3'd7: busX_r = r7_w;
        default: busX_r = 8'b0000_0000;
    endcase

    case (RY)
        3'd0: busY_r = 8'b0000_0000;
        3'd1: busY_r = r1_w;
        3'd2: busY_r = r2_w;
        3'd3: busY_r = r3_w;
        3'd4: busY_r = r4_w;
        3'd5: busY_r = r5_w;
        3'd6: busY_r = r6_w;
        3'd7: busY_r = r7_w;
        default: busY_r = 8'b0000_0000;
    endcase
end

always @(posedge Clk) begin
    // keep r0 hard-wired to zero regardless of RW/busW
    r0_w <= 8'b0000_0000;

    if (WEN) begin
        case (RW)
            3'd0: r0_w <= 8'b0000_0000;
            3'd1: r1_w <= busW;
            3'd2: r2_w <= busW;
            3'd3: r3_w <= busW;
            3'd4: r4_w <= busW;
            3'd5: r5_w <= busW;
            3'd6: r6_w <= busW;
            3'd7: r7_w <= busW;
            default: ;
        endcase
    end
end

endmodule
