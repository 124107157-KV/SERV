`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 06:59:12
// Design Name: 
// Module Name: tb_serv_decode
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


`timescale 1ns / 1ps
`default_nettype none

module tb_serv_decode;

    // Parameters
    parameter PRE_REGISTER = 1;
    parameter MDU = 1;

    // Inputs
    reg clk;
    reg i_wb_en;
    reg [31:2] i_wb_rdt;

    // Outputs
    wire o_rd_alu_en;
    wire o_rd_mem_en;
    wire o_branch_op;
    wire o_shift_op;
    wire o_mdu_op;

    // Instantiate DUT
    serv_decode #(
        .PRE_REGISTER(PRE_REGISTER),
        .MDU(MDU)
    ) dut (
        .clk(clk),
        .i_wb_en(i_wb_en),
        .i_wb_rdt(i_wb_rdt),
        .o_rd_alu_en(o_rd_alu_en),
        .o_rd_mem_en(o_rd_mem_en),
        .o_branch_op(o_branch_op),
        .o_shift_op(o_shift_op),
        .o_mdu_op(o_mdu_op)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // Test Procedure
    initial begin
        clk = 0;
        i_wb_en = 1;

        // Load Instruction
        i_wb_rdt = 30'b000000000000000000000000000000;
        #10;

        // Branch Instruction
        i_wb_rdt = 30'b000000000000000000000000110000;
        #10;

        // ALU Instruction
        i_wb_rdt = 30'b000000000000000000000001100000;
        #10;

        // End Simulation
        #50 $finish;
    end

endmodule

