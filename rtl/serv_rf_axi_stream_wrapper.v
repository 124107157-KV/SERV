`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.03.2025 04:05:49
// Design Name: 
// Module Name: serv_rf_axi_stream_wrapper
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


//==============================================================================
// Top-Level Wrapper Module Using AXI4-Stream Interfaces
// This module instantiates the SERV core and uses protocol conversion wrappers
// to convert between a narrow AXI4-Stream interface (external) and the wide
// parallel bus used internally by serv_rf_top.
//==============================================================================
module serv_rf_axi_stream_wrapper
  #(
    parameter RESET_PC      = 32'd0,
    parameter [0:0] COMPRESSED   = 0,
    parameter [0:0] ALIGN        = COMPRESSED,
    parameter [0:0] MDU          = 0,
    parameter PRE_REGISTER  = 1,
    parameter RESET_STRATEGY = "MINI",
    parameter [0:0] DEBUG        = 1'b0,
    parameter WITH_CSR       = 1,
    parameter W              = 1,
    parameter RF_WIDTH       = W * 2,
    parameter RF_L2D         = $clog2((32+(WITH_CSR*4))*32/RF_WIDTH),
    parameter USE_STOC_ALU   = 1,
    // Define the internal parallel bus width (must match the bundling in serv_rf_top)
    parameter DATA_WIDTH = 171
  )
  (
    // AXI4-Stream Slave Interface for Write (external input)
    input  wire         s_tvalid,
    input  wire [31:0]  s_tdata,
    input  wire         s_tlast,
    output wire         s_tready,
    
    // AXI4-Stream Master Interface for Read (external output)
    output wire         m_tvalid,
    output wire [31:0]  m_tdata,
    output wire         m_tlast,
    input  wire         m_tready,
    
    // Optional external clock/reset (if not provided, use internal)
    input  wire         clk,
    input  wire         rst
  );

  // If external clock/reset are not provided, you can instantiate internal ones.
  // For simplicity, we assume clk and rst are provided.
  
  // Internal parallel bus signals connecting to serv_rf_top
  wire [DATA_WIDTH-1:0] parallel_out; // from SERV core (outputs)
  wire [DATA_WIDTH-1:0] parallel_in;  // to SERV core (inputs)
  
  // Internal dummy signals for SERV core unused ports.
  wire int_dummy;
  
  // Instantiate the SERV core (serv_rf_top) with bundled I/Os.
  // For the AXI conversion, we assume that the SERV core has been modified to pack
  // its external signals into a single parallel bus.
  serv_rf_top
    #(
      .RESET_PC       (RESET_PC),
      .COMPRESSED     (COMPRESSED),
      .ALIGN          (ALIGN),
      .MDU            (MDU),
      .PRE_REGISTER   (PRE_REGISTER),
      .RESET_STRATEGY (RESET_STRATEGY),
      .DEBUG          (DEBUG),
      .WITH_CSR       (WITH_CSR),
      .W              (W),
      .RF_WIDTH       (RF_WIDTH),
      .RF_L2D         (RF_L2D),
      .USE_STOC_ALU   (USE_STOC_ALU)
    )
  serv_core_inst
    (
      .clk         (clk),
      .i_rst       (rst),
      .i_timer_irq (1'b0),
      // For this example, we assume the SERV core's external signals are bundled as follows:
      .o_ibus_adr  (parallel_out[DATA_WIDTH-1:DATA_WIDTH-32]),
      .o_ibus_cyc  (parallel_out[DATA_WIDTH-33]),
      .i_ibus_rdt  (parallel_in[DATA_WIDTH-1:DATA_WIDTH-32]),
      .i_ibus_ack  (parallel_in[DATA_WIDTH-33]),
      .o_dbus_adr  (parallel_out[DATA_WIDTH-34:DATA_WIDTH-65]),
      .o_dbus_dat  (parallel_out[DATA_WIDTH-66:DATA_WIDTH-97]),
      .o_dbus_sel  (parallel_out[DATA_WIDTH-98:DATA_WIDTH-101]),
      .o_dbus_we   (parallel_out[DATA_WIDTH-102]),
      .o_dbus_cyc  (parallel_out[DATA_WIDTH-103]),
      .i_dbus_rdt  (parallel_in[DATA_WIDTH-34:DATA_WIDTH-65]),
      .i_dbus_ack  (parallel_in[DATA_WIDTH-102]),
      .o_mdu_valid (parallel_out[0])
    );
    
  // Instantiate the AXI4-Stream to Parallel converter (for write direction)
  axi_stream_to_parallel #(
      .DATA_WIDTH_PAR(DATA_WIDTH),
      .WORDS(6),
      .DATA_WIDTH_STR(32)
  ) write_conv (
      .clk         (clk),
      .rst         (rst),
      .s_tdata     (s_tdata),
      .s_tvalid    (s_tvalid),
      .s_tready    (s_tready),
      .s_tlast     (s_tlast),
      .parallel_out(parallel_in),
      .valid_out   (/* can be used for handshake internally; not used here */)
  );
  
  // Instantiate the Parallel to AXI4-Stream converter (for read direction)
  parallel_to_axi_stream #(
      .DATA_WIDTH_PAR(DATA_WIDTH),
      .WORDS(6),
      .DATA_WIDTH_STR(32)
  ) read_conv (
      .clk         (clk),
      .rst         (rst),
      .parallel_in (parallel_out),
      .valid_in    (1'b1), // Assume always valid data for demonstration
      .m_tdata     (m_tdata),
      .m_tvalid    (m_tvalid),
      .m_tready    (m_tready),
      .m_tlast     (m_tlast)
  );
  
endmodule

`default_nettype wire