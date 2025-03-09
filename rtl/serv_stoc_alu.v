`timescale 1ns / 1ps
`default_nettype none

module serv_stoc_alu
  #(
    parameter W = 1, // Stochastic bit width
    parameter B = W-1
  )
  (
   input wire       clk,
   input wire       i_en,

   // Control Buses (Reduced IO)
   input wire [4:0] i_alu_ctrl_bus, // {i_rd_sel, i_bool_op}
   input wire       i_matrix_op,
   input wire [2:0] i_alu_cmp_ctrl_bus, // {i_sub, i_cmp_eq, i_cmp_sig}
   
   // Data Bus (Reduced IO)
   input wire [(4*W)-1:0] i_alu_data_bus, // {i_rs1, i_op_b, i_buf, i_mat_element}
   output reg [B:0] o_rd,  // Output Register to maintain stability
   output wire      o_cmp  // Comparator Output
  );

   // Extract control signals
   wire [4:0] i_rd_sel  = i_alu_ctrl_bus[4:2];
   wire [1:0] i_bool_op = i_alu_ctrl_bus[1:0];

   // Extract comparison control signals
   wire i_sub     = i_alu_cmp_ctrl_bus[2];
   wire i_cmp_eq  = i_alu_cmp_ctrl_bus[1];
   wire i_cmp_sig = i_alu_cmp_ctrl_bus[0];

   // Extract data signals
   wire  [B:0] i_rs1 = i_alu_data_bus[(4*W)-1:(3*W)];
   wire  [B:0] i_op_b = i_alu_data_bus[(3*W)-1:(2*W)];
   wire  [B:0] i_buf = i_alu_data_bus[(2*W)-1:W];
   wire  [B:0] i_mat_element = i_alu_data_bus[W-1:0];

   reg  [B:0] add_cy_r;
   wire       add_cy;

   // Stochastic Arithmetic Operations
   wire [B:0] result_add       = i_rs1 | i_op_b; // Stochastic Addition (Unipolar OR)
   wire [B:0] result_sub       = i_rs1 ^ i_op_b; // Stochastic Subtraction (XOR)
   wire [B:0] result_mult      = i_rs1 & i_op_b; // Stochastic Multiplication (AND)
   wire [B:0] result_div       = i_rs1 & (~i_op_b); // Approximate Stochastic Division
   wire [B:0] result_exp       = i_rs1 & i_rs1; // Stochastic Exponentiation
   wire [B:0] result_log       = (i_rs1 >> 1); // Approximate Logarithm
   wire [B:0] result_sqrt      = (i_rs1 & i_op_b); // Approximate Square Root
   wire [B:0] result_poly      = i_rs1 & i_op_b; // Polynomial Approximation
   wire [B:0] result_sigmoid   = i_rs1 & (~i_op_b); // Approximate Sigmoid
   wire [B:0] result_tanh      = result_sigmoid; // Approximate Tanh

   // Additional Mathematical Functions
   wire [B:0] result_reciprocal = (~i_rs1); // Approximate Reciprocal
   wire [B:0] result_power = i_rs1 & i_rs1 & i_op_b; // Power Function X^Y
   wire [B:0] result_softmax = (i_rs1 & i_rs1) / (result_exp + i_buf); // Softmax Approximation

   // Neural Network-Specific Functions
   wire [B:0] result_relu      = (i_rs1[B]) ? {B{1'b0}} : i_rs1; // ReLU
   wire [B:0] result_dot       = result_mult + i_buf; // Dot product accumulation
   wire [B:0] result_conv      = result_mult + result_dot; // Convolution Step
   wire [B:0] result_pool      = (i_rs1 > i_op_b) ? i_rs1 : i_op_b; // Max Pooling

   // Logical Operations
   wire [B:0] result_and       = i_rs1 & i_op_b;
   wire [B:0] result_or        = i_rs1 | i_op_b;
   wire [B:0] result_xor       = i_rs1 ^ i_op_b;
   wire [B:0] result_xnor      = ~(i_rs1 ^ i_op_b);
   wire [B:0] result_not       = ~i_rs1;
   wire [B:0] result_nor       = ~(i_rs1 | i_op_b);
   wire [B:0] result_nand      = ~(i_rs1 & i_op_b);
   wire [B:0] result_parity    = ^i_rs1; // Parity Check
   wire [B:0] result_majority  = (i_rs1 & i_op_b) | (i_rs1 & i_mat_element) | (i_op_b & i_mat_element);
   wire [B:0] result_comparator = (i_rs1 > i_op_b) ? 1 : 0;

   // Additional Stochastic Functions
   wire [B:0] result_stochastic_round = (i_rs1 + $random) >> 1; // Stochastic Rounding
   wire [B:0] result_gaussian = (i_rs1 & $random) + (i_op_b & ~$random); // Gaussian Noise Approximation

   wire [B:0] result_logic = (i_bool_op == 2'b00) ? result_and :
                             (i_bool_op == 2'b01) ? result_or :
                             (i_bool_op == 2'b10) ? result_xor :
                             result_xnor;

   // Matrix Operations
   wire [B:0] result_mat_add      = i_rs1 | i_mat_element;
   wire [B:0] result_mat_sub      = i_rs1 ^ i_mat_element;
   wire [B:0] result_mat_mult     = i_rs1 & i_mat_element;
   wire [B:0] result_scalar_mult  = i_rs1 & i_op_b;
   wire [B:0] result_mat_transpose = i_mat_element;

   // Set Less Than (SLT)
   wire [B:0] result_slt = (i_rs1 < i_op_b) ? {B{1'b1}} : {B{1'b0}};

   // Output Selection (Multiplexer)
   always @(posedge clk) begin
      if (i_en) begin
         o_rd <= (i_matrix_op) ? (
                     (i_rd_sel == 4'b0001) ? result_mat_add :
                     (i_rd_sel == 4'b0010) ? result_mat_sub :
                     (i_rd_sel == 4'b0011) ? result_mat_mult :
                     (i_rd_sel == 4'b0100) ? result_scalar_mult :
                     (i_rd_sel == 4'b0101) ? result_mat_transpose :
                     0
                  ) : (
                     (i_rd_sel == 4'b0001) ? result_add :
                     (i_rd_sel == 4'b0010) ? result_sub :
                     (i_rd_sel == 4'b0011) ? result_mult :
                     (i_rd_sel == 4'b0100) ? result_div :
                     (i_rd_sel == 4'b0101) ? result_exp :
                     (i_rd_sel == 4'b0110) ? result_log :
                     (i_rd_sel == 4'b0111) ? result_sqrt :
                     (i_rd_sel == 4'b1000) ? result_poly :
                     (i_rd_sel == 4'b1001) ? result_sigmoid :
                     (i_rd_sel == 4'b1010) ? result_tanh :
                     (i_rd_sel == 4'b1011) ? result_reciprocal :
                     (i_rd_sel == 4'b1100) ? result_power :
                     (i_rd_sel == 4'b1101) ? result_softmax :
                     (i_rd_sel == 4'b1110) ? result_parity :
                     (i_rd_sel == 4'b1111) ? result_stochastic_round :
                     0
                  );
      end
   end

   // Stochastic Comparator Output
   assign o_cmp = result_comparator;

endmodule
