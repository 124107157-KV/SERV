`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 07:05:17
// Design Name: 
// Module Name: tb_serv_immdec
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

module tb_serv_immdec;
  
    parameter SHARED_RFADDR_IMM_REGS = 1;

    // Inputs
    reg clk;
    reg i_wb_en;
    reg i_cnt_en;
    reg i_cnt_done;
    reg [3:0] i_immdec_en;
    reg i_csr_imm_en;
    reg [3:0] i_ctrl;
    reg [31:7] i_wb_rdt;

    // Outputs
    wire [4:0] o_rd_addr;
    wire [4:0] o_rs1_addr;
    wire [4:0] o_rs2_addr;
    wire o_csr_imm;
    wire o_imm;

    // Instantiate DUT
    serv_immdec #(.SHARED_RFADDR_IMM_REGS(SHARED_RFADDR_IMM_REGS)) dut (
        .i_clk(clk),
        .i_wb_en(i_wb_en),
        .i_cnt_en(i_cnt_en),
        .i_cnt_done(i_cnt_done),
        .i_immdec_en(i_immdec_en),
        .i_csr_imm_en(i_csr_imm_en),
        .i_ctrl(i_ctrl),
        .i_wb_rdt(i_wb_rdt),
        .o_rd_addr(o_rd_addr),
        .o_rs1_addr(o_rs1_addr),
        .o_rs2_addr(o_rs2_addr),
        .o_csr_imm(o_csr_imm),
        .o_imm(o_imm)
    );

    always #5 clk = ~clk;

    initial begin
        clk = 0;
        i_wb_en = 1;

        // Test Load Immediate (I-Type)
        i_wb_rdt = 25'b0000000000000000000000001;
        #10;

        // Test Store Immediate (S-Type)
        i_wb_rdt = 25'b1111111111111111111111111;
        #10;

        $finish;
    end
endmodule
