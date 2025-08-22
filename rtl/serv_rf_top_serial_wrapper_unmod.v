`timescale 1ns / 1ps
`default_nettype none

//---------------------------------------------------------------------
// This wrapper instantiates the full serv_rf_top core but then 
// converts its many parallel I/Os into a narrow serial interface.
// The external interface uses only a few pins:
//   - ser_clk  : serial clock output
//   - ser_data_out : serial data out
//   - ser_data_in  : serial data in
//   - ser_cs       : chip-select (active low)
//---------------------------------------------------------------------

module serv_rf_top_serial_wrapper (
    // System clock & reset
    input  wire        clk,
    input  wire        rst,
    input  wire        timer_irq,
    
    // External serial interface (reduces IO count drastically)
    output wire        ser_clk,
    output wire        ser_data_out,
    input  wire        ser_data_in,
    output wire        ser_cs
);

  //--------------------------------------------------------------------------
  // Internal Parallel Bus Signals
  //--------------------------------------------------------------------------
  // Assume that the complete serv_rf_top external interface (after IO bundling)
  // is N bits wide. For this example we set DATA_WIDTH = 171.
  parameter DATA_WIDTH = 171;
  
  // We define two buses:
  // - parallel_out: the outputs from serv_rf_top (to be transmitted serially)
  // - parallel_in : the inputs that serv_rf_top expects (received serially)
  wire [DATA_WIDTH-1:0] parallel_out;
  wire [DATA_WIDTH-1:0] parallel_in;
  
  //--------------------------------------------------------------------------
  // Instantiate the serv_rf_top Core
  //--------------------------------------------------------------------------
  // In this example, we disable the RVFI and EXT interfaces so that they do not
  // require external pins. All I/Os are bundled into a single wide bus.
  // (The internal implementation of bundling may be achieved by modifying serv_rf_top;
  // here we assume that a separate module "serv_rf_top_parallel" packs the signals.)
  serv_rf_top #(
    .RESET_PC       (32'd0),
    .COMPRESSED     (0),
    .ALIGN          (0),
    .MDU            (0),
    .PRE_REGISTER   (1),
    .RESET_STRATEGY ("MINI"),
    .DEBUG          (1'b0),
    .WITH_CSR       (1),
    .W              (1),
    .RF_WIDTH       (1 * 2),
    .RF_L2D         ($clog2((32+(1*4))*32/(1*2))),
    .USE_STOC_ALU   (1)
  ) u_serv_rf_top (
    .clk         (clk),
    .i_rst       (rst),
    .i_timer_irq (timer_irq),
`ifdef RVFI_ENABLED    
    // Tie off RVFI and EXT ports
    .rvfi_valid     (),
    .rvfi_order     (),
    .rvfi_insn      (),
    .rvfi_trap      (),
    .rvfi_halt      (),
    .rvfi_intr      (),
    .rvfi_mode      (),
    .rvfi_ixl       (),
    .rvfi_rs1_addr  (),
    .rvfi_rs2_addr  (),
    .rvfi_rs1_rdata (),
    .rvfi_rs2_rdata (),
    .rvfi_rd_addr   (),
    .rvfi_rd_wdata  (),
    .rvfi_pc_rdata  (),
    .rvfi_pc_wdata  (),
    .rvfi_mem_addr  (),
    .rvfi_mem_rmask (),
    .rvfi_mem_wmask (),
    .rvfi_mem_rdata (),
    .rvfi_mem_wdata (),
`endif
    // Instead of driving separate instruction and data buses,
    // we assume that a bus-consolidation module packs all the external signals
    // into a single wide bus. Here, we connect these ports to our parallel bus.
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
`ifdef EXT_ENABLED    
    // For EXT interface, tie off signals
    .o_ext_rs1   (),
    .o_ext_rs2   (),
    .o_ext_funct3(),
    .i_ext_rd    (32'd0),
    .i_ext_ready (1'b1),
`endif    
    .o_mdu_valid (parallel_out[0]) // Example: map MDU valid to one bit
  );
  
  //--------------------------------------------------------------------------
  // Instantiate the Parallel-to-Serial Interface Module
  //--------------------------------------------------------------------------
  // This module converts the wide parallel bus (both input and output) into a
  // serial stream over a few pins.
  parallel_serial_if #(
    .DATA_WIDTH(DATA_WIDTH)
  ) ps_if (
    .clk         (clk),
    .rst         (rst),
    .parallel_in (parallel_out),
    .parallel_out(parallel_in),
    .ser_clk     (ser_clk),
    .ser_data_out(ser_data_out),
    .ser_data_in (ser_data_in),
    .ser_cs      (ser_cs)
  );

endmodule

`default_nettype wire
