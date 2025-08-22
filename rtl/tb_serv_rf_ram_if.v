`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 06:41:56
// Design Name: 
// Module Name: tb_serv_rf_ram_if
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

module tb_serv_rf_ram_if;

    parameter width = 8;
    parameter W = 1;
    parameter reset_strategy = "MINI";
    parameter csr_regs = 4;

    reg  i_clk;
    reg  i_rst;
    reg  i_wreq;
    reg  i_rreq;
    wire o_ready;
    reg  [4:0] i_wreg0;
    reg  [4:0] i_wreg1;
    reg  i_wen0;
    reg  i_wen1;
    reg  [W-1:0] i_wdata0;
    reg  [W-1:0] i_wdata1;
    reg  [4:0] i_rreg0;
    reg  [4:0] i_rreg1;
    wire [W-1:0] o_rdata0;
    wire [W-1:0] o_rdata1;
    reg  [width-1:0] i_rdata;
    
    wire [4:0] o_waddr;
    wire [width-1:0] o_wdata;
    wire o_wen;
    wire [4:0] o_raddr;
    wire o_ren;
    
    // Instantiate DUT
    serv_rf_ram_if #(
        .width(width),
        .W(W),
        .reset_strategy(reset_strategy),
        .csr_regs(csr_regs)
    ) dut (
        .i_clk(i_clk),
        .i_rst(i_rst),
        .i_wreq(i_wreq),
        .i_rreq(i_rreq),
        .o_ready(o_ready),
        .i_wreg0(i_wreg0),
        .i_wreg1(i_wreg1),
        .i_wen0(i_wen0),
        .i_wen1(i_wen1),
        .i_wdata0(i_wdata0),
        .i_wdata1(i_wdata1),
        .i_rreg0(i_rreg0),
        .i_rreg1(i_rreg1),
        .o_rdata0(o_rdata0),
        .o_rdata1(o_rdata1),
        .o_waddr(o_waddr),
        .o_wdata(o_wdata),
        .o_wen(o_wen),
        .o_raddr(o_raddr),
        .o_ren(o_ren),
        .i_rdata(i_rdata)
    );

    // Clock Generation
    always #5 i_clk = ~i_clk;

    // Test Procedure
    initial begin
        i_clk = 0;
        i_rst = 1;
        i_wreq = 0;
        i_rreq = 0;
        i_wreg0 = 5'b00001;
        i_wreg1 = 5'b00010;
        i_wen0 = 0;
        i_wen1 = 0;
        i_wdata0 = 1'b1;
        i_wdata1 = 1'b0;
        i_rreg0 = 5'b00001;
        i_rreg1 = 5'b00010;
        i_rdata = 8'hFF;

        #10 i_rst = 0;

        // Write test
        #10 i_wreq = 1; i_wen0 = 1; i_wdata0 = 1'b1;
        #10 i_wreq = 0; i_wen0 = 0;

        // Read test
        #10 i_rreq = 1;
        #10 i_rreq = 0;

        // End simulation
        #50 $finish;
    end

endmodule
