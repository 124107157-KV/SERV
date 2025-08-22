`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 06:45:47
// Design Name: 
// Module Name: tb_serv_rf_ram
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

module tb_serv_rf_ram;

    parameter width = 8;
    parameter csr_regs = 4;
    parameter depth = 32*(32+csr_regs)/width;

    reg  i_clk;
    reg  i_wen;
    reg  i_ren;
    reg  [$clog2(depth)-1:0] i_waddr;
    reg  [$clog2(depth)-1:0] i_raddr;
    reg  [width-1:0] i_wdata;
    wire [width-1:0] o_rdata;

    // Instantiate DUT
    serv_rf_ram #(
        .width(width),
        .csr_regs(csr_regs),
        .depth(depth)
    ) dut (
        .i_clk(i_clk),
        .i_waddr(i_waddr),
        .i_wdata(i_wdata),
        .i_wen(i_wen),
        .i_raddr(i_raddr),
        .i_ren(i_ren),
        .o_rdata(o_rdata)
    );

    // Clock Generation
    always #5 i_clk = ~i_clk;

    // Test Procedure
    initial begin
        i_clk = 0;
        i_wen = 0;
        i_ren = 0;
        i_waddr = 5'b00001;
        i_wdata = 8'hAA; // Test data
        i_raddr = 5'b00001;

        // Write Test
        #10 i_wen = 1;
        #10 i_wen = 0;

        // Read Test
        #10 i_ren = 1;
        #10 i_ren = 0;

        // Register x0 Read Test
        #10 i_raddr = 5'b00000; // Should always return 0
        #10 i_ren = 1;
        #10 i_ren = 0;

        // End Simulation
        #50 $finish;
    end

endmodule
