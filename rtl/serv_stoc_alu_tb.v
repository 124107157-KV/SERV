`timescale 1ns / 1ps
`default_nettype none

module serv_stoc_alu_tb;

  parameter W = 8; // Test with an 8-bit stochastic representation
  parameter B = W-1;

  reg clk;
  reg i_en;
  reg [4:0] i_alu_ctrl_bus;
  reg i_matrix_op;
  reg [2:0] i_alu_cmp_ctrl_bus;
  reg [(4*W)-1:0] i_alu_data_bus;
  wire [B:0] o_rd;
  wire o_cmp;

  // Instantiate the ALU module
  serv_stoc_alu #(W) uut (
    .clk(clk),
    .i_en(i_en),
    .i_alu_ctrl_bus(i_alu_ctrl_bus),
    .i_matrix_op(i_matrix_op),
    .i_alu_cmp_ctrl_bus(i_alu_cmp_ctrl_bus),
    .i_alu_data_bus(i_alu_data_bus),
    .o_rd(o_rd),
    .o_cmp(o_cmp)
  );

  // Clock generation
  always #5 clk = ~clk;

  initial begin
    // Initialize inputs
    clk = 0;
    i_en = 0;
    i_alu_ctrl_bus = 5'b00000;
    i_matrix_op = 0;
    i_alu_cmp_ctrl_bus = 3'b000;
    i_alu_data_bus = 0;

    #10;
    
    // Enable ALU
    i_en = 1;
    
    // Debugging - Monitor values
    $monitor("Time=%0t, Ctrl=%b, DataBus=%h, Output=%h, Comparator=%b",
              $time, i_alu_ctrl_bus, i_alu_data_bus, o_rd, o_cmp);
    
    // Test ReLU
    $display("Testing ReLU");
    i_alu_ctrl_bus = 5'b00001; 
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b10000000;
    #10;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00100000;
    #10;
    
    // Test Leaky ReLU
    $display("Testing Leaky ReLU");
    i_alu_ctrl_bus = 5'b00010;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b10000000;
    #10;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00100000;
    #10;
    
    // Test Swish Activation
    $display("Testing Swish Activation");
    i_alu_ctrl_bus = 5'b00011;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b10101010;
    #10;
    
    // Test Pooling (Max and Avg)
    $display("Testing Pooling Max and Avg");
    i_alu_ctrl_bus = 5'b00100;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b01010101;
    i_alu_data_bus[(3*W)-1:(2*W)] = 8'b01100000;
    #10;
    
    // Test Adaptive Pooling
    $display("Testing Adaptive Pooling");
    i_alu_ctrl_bus = 5'b00101;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b01111000;
    #10;
    
    // Test Convolution
    $display("Testing Convolution");
    i_alu_ctrl_bus = 5'b00110;
    i_matrix_op = 1;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00101100;
    i_alu_data_bus[(3*W)-1:(2*W)] = 8'b00011110;
    i_alu_data_bus[(2*W)-1:W] = 8'b00010101;
    i_alu_data_bus[W-1:0] = 8'b00110011;
    #10;
    
    // Test Softmax
    $display("Testing Softmax");
    i_alu_ctrl_bus = 5'b00111;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00101010;
    #10;
    
    // Test Batch Normalization
    $display("Testing Batch Normalization");
    i_alu_ctrl_bus = 5'b01000;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00110101;
    #10;
    
    // Test Dropout
    $display("Testing Dropout");
    i_alu_ctrl_bus = 5'b01001;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b11110000;
    #10;
    
    // Test Self-Attention
    $display("Testing SSelf-Attention");
    i_alu_ctrl_bus = 5'b01010;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00101111;
    #10;
    
    // Test LSTM/GRU Computation
    $display("Testing LSTM / GRU Computation");
    i_alu_ctrl_bus = 5'b01011;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b00111001;
    i_alu_data_bus[(3*W)-1:(2*W)] = 8'b00110010;
    #10;
    
    // Test Logical Operations (AND, OR, XOR, XNOR)
    $display("Testing AND OR XOR XNOR");
    i_alu_ctrl_bus = 5'b01110;
    i_alu_data_bus = 32'hAA55AA55;
    #10;
    i_alu_ctrl_bus = 5'b01111;
    i_alu_data_bus = 32'h55AA55AA;
    #10;
    i_alu_ctrl_bus = 5'b10000;
    #10;
    i_alu_ctrl_bus = 5'b10001;
    #10;
    
    // Test Stochastic Rounding and Gaussian Noise Approximation
    $display("Testing Stochastic Rounding and Gaussian Noise Approximation");
    i_alu_ctrl_bus = 5'b10010;
    i_alu_data_bus = 32'h12345678;
    #10;
    i_alu_ctrl_bus = 5'b10011;
    #10;
    
    // Test Comparator
    $display("Testing Comparator");
    i_alu_ctrl_bus = 5'b01100;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b01011010;
    i_alu_data_bus[(3*W)-1:(2*W)] = 8'b01100000;
    #10;
    
    // Test Weight Prefetching
    $display("Testing Weight Prefetching");
    i_alu_ctrl_bus = 5'b01101;
    i_alu_data_bus[(4*W)-1:(3*W)] = 8'b10101010;
    #10;
    
    // New Activation and Approximation Functions
    $display("Testing Sigmoid Approximation");
    i_alu_ctrl_bus = 5'b10100;
    i_alu_data_bus = 32'h5A601533;
    #10;
    
    $display("Testing Tanh Approximation");
    i_alu_ctrl_bus = 5'b10101;
    i_alu_data_bus = 32'hAA605678;
    #10;
    
    $display("Testing Reciprocal Approximation");
    i_alu_ctrl_bus = 5'b10110;
    i_alu_data_bus = 32'h12345678;
    #10;
    
    $display("Testing Power Function");
    i_alu_ctrl_bus = 5'b10111;
    i_alu_data_bus = 32'hFEDCBA98;
    #10;
    
    $display("Testing Gaussian Noise Approximation");
    i_alu_ctrl_bus = 5'b11000;
    i_alu_data_bus = 32'h13579BDF;
    #10;
    
    // End simulation
    $stop;
  end

endmodule
