`timescale 1ns/10ps

// Generic compute for any row i (boundary handled by feeding 0s):
// x_i = ( b_i
//         + 13*(x_{i-1}+x_{i+1})
//         -  6*(x_{i-2}+x_{i+2})
//         +  1*(x_{i-3}+x_{i+3}) ) / 20
//
// Formats:
// - bi: 16-bit signed integer (2's complement)
// - x_*: 32-bit signed Q16.16
// - x_out: 32-bit signed Q16.16
module core_xi (
  input  wire signed [15:0] bi,
  input  wire signed [31:0] x_im3,
  input  wire signed [31:0] x_im2,
  input  wire signed [31:0] x_im1,
  input  wire signed [31:0] x_ip1,
  input  wire signed [31:0] x_ip2,
  input  wire signed [31:0] x_ip3,
  output wire signed [31:0] x_out
);

  // Sign-extend to wider bitwidth for safe shift-add and accumulation.
  wire signed [47:0] bi_q16  = {{32{bi[15]}},  bi}  <<< 16; // int -> Q16.16 in 48b
  wire signed [47:0] xim3_48 = {{16{x_im3[31]}}, x_im3};
  wire signed [47:0] xim2_48 = {{16{x_im2[31]}}, x_im2};
  wire signed [47:0] xim1_48 = {{16{x_im1[31]}}, x_im1};
  wire signed [47:0] xip1_48 = {{16{x_ip1[31]}}, x_ip1};
  wire signed [47:0] xip2_48 = {{16{x_ip2[31]}}, x_ip2};
  wire signed [47:0] xip3_48 = {{16{x_ip3[31]}}, x_ip3};

  // Constant multipliers (shift-add).
  wire signed [47:0] sum_p13 = (xim1_48 + xip1_48);
  wire signed [47:0] sum_m6  = (xim2_48 + xip2_48);
  wire signed [47:0] sum_p1  = (xim3_48 + xip3_48);

  wire signed [47:0] term_p13 = sum_p13 + (sum_p13 <<< 2) + (sum_p13 <<< 3); // *13
  wire signed [47:0] term_m6  = -((sum_m6  <<< 1) + (sum_m6  <<< 2));        // *(-6)
  wire signed [47:0] term_p1  = sum_p1;

  wire signed [47:0] numer = bi_q16 + term_p13 + term_m6 + term_p1; // Q16.16

  // First version: exact constant divide by 20 (synthesizable in most flows).
  // Truncates toward zero (Verilog signed division behavior).
  wire signed [47:0] div20 = numer / 48'sd20;

  assign x_out = div20[31:0];

endmodule

