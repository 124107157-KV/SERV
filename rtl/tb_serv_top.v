`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 06:50:37
// Design Name: 
// Module Name: tb_serv_top
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

module tb_serv_top;

    // Parameters
    parameter WITH_CSR = 1;
    parameter W = 1;
    parameter USE_STOC_ALU = 1;

    // Clock and Reset
    reg clk;
    reg i_rst;
    reg i_timer_irq;

    // Register File Interface
    wire o_rf_rreq;
    wire o_rf_wreq;
    reg i_rf_ready;
    wire [4+WITH_CSR:0] o_wreg0;
    wire [4+WITH_CSR:0] o_wreg1;
    wire o_wen0;
    wire o_wen1;
    wire [W-1:0] o_wdata0;
    wire [W-1:0] o_wdata1;
    wire [4+WITH_CSR:0] o_rreg0;
    wire [4+WITH_CSR:0] o_rreg1;
    reg [W-1:0] i_rdata0;
    reg [W-1:0] i_rdata1;

    // Instantiate DUT
    serv_top #(
        .WITH_CSR(WITH_CSR),
        .W(W),
        .USE_STOC_ALU(USE_STOC_ALU)
    ) dut (
        .clk(clk),
        .i_rst(i_rst),
        .i_timer_irq(i_timer_irq),
        .o_rf_rreq(o_rf_rreq),
        .o_rf_wreq(o_rf_wreq),
        .i_rf_ready(i_rf_ready),
        .o_wreg0(o_wreg0),
        .o_wreg1(o_wreg1),
        .o_wen0(o_wen0),
        .o_wen1(o_wen1),
        .o_wdata0(o_wdata0),
        .o_wdata1(o_wdata1),
        .o_rreg0(o_rreg0),
        .o_rreg1(o_rreg1),
        .i_rdata0(i_rdata0),
        .i_rdata1(i_rdata1)
    );

    // Clock Generation
    always #5 clk = ~clk;

    // Test Procedure
    initial begin
        clk = 0;
        i_rst = 1;
        #20 i_rst = 0;
        #100 $finish;
    end

endmodule
