`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 06:55:53
// Design Name: 
// Module Name: tb_serv_state
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

module tb_serv_state;

    // Parameters
    parameter RESET_STRATEGY = "MINI";
    parameter WITH_CSR = 1;
    parameter MDU = 1;
    parameter W = 1;

    // Clock and Reset
    reg i_clk;
    reg i_rst;
    reg i_new_irq;
    reg i_alu_cmp;
    reg i_mdu_op;
    reg i_mdu_ready;
    reg i_dbus_en;
    reg i_branch_op;
    reg i_cond_branch;
    reg i_bne_or_bge;

    // Outputs
    wire o_init;
    wire o_cnt_en;
    wire o_rf_rreq;
    wire o_rf_wreq;
    wire o_mdu_valid;

    // Instantiate DUT
    serv_state #(
        .RESET_STRATEGY(RESET_STRATEGY),
        .WITH_CSR(WITH_CSR),
        .MDU(MDU),
        .W(W)
    ) dut (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_new_irq(i_new_irq),
        .i_alu_cmp(i_alu_cmp),
        .i_mdu_op(i_mdu_op),
        .i_mdu_ready(i_mdu_ready),
        .i_dbus_en(i_dbus_en),
        .i_branch_op(i_branch_op),
        .i_cond_branch(i_cond_branch),
        .i_bne_or_bge(i_bne_or_bge),
        .o_init(o_init),
        .o_cnt_en(o_cnt_en),
        .o_rf_rreq(o_rf_rreq),
        .o_rf_wreq(o_rf_wreq),
        .o_mdu_valid(o_mdu_valid)
    );

    // Clock Generation
    always #5 i_clk = ~i_clk;

    // Test Procedure
    initial begin
        i_clk = 0;
        i_rst = 1;
        i_new_irq = 0;
        i_mdu_op = 0;
        i_mdu_ready = 1;
        i_dbus_en = 0;
        i_branch_op = 0;
        i_cond_branch = 0;
        i_bne_or_bge = 0;

        #20 i_rst = 0;

        // Simulate branch execution
        #10 i_branch_op = 1; i_alu_cmp = 1;
        #10 i_branch_op = 0;

        // Simulate MDU execution
        #10 i_mdu_op = 1;
        #10 i_mdu_op = 0;

        // End simulation
        #100 $finish;
    end

endmodule
