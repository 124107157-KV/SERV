`timescale 1ns / 1ps
`default_nettype none

//---------------------------------------------------------------------
// Top-Level Module with No External IO
// All logic is kept internal, so IO utilization is 0.
//---------------------------------------------------------------------
(* DONT_TOUCH = "true" *)
module serv_rf_top_serial_wrapper_noio();

  // ---------------------------------------------------------------
  // 1. Internal Clock and Reset
  // ---------------------------------------------------------------
  reg clk_reg = 1'b0;
  always #5 clk_reg = ~clk_reg; // ~100 MHz toggling clock

  reg rst_reg = 1'b1;
  initial begin
    // Keep reset active for 200 ns
    #200 rst_reg = 1'b0;
  end

  // We'll tie off the timer_irq signal (not needed externally)
  wire timer_irq = 1'b0;

  // ---------------------------------------------------------------
  // 2. Internal Wires for Parallel Buses
  // ---------------------------------------------------------------
  parameter DATA_WIDTH = 171;
  
  // The wide parallel buses for input/output
  wire [DATA_WIDTH-1:0] parallel_out;
  wire [DATA_WIDTH-1:0] parallel_in;

  // ---------------------------------------------------------------
  // 3. Instantiate the SERV Core
  // ---------------------------------------------------------------
  (* DONT_TOUCH = "true" *)
  serv_rf_top #(
    .RESET_PC       (32'd0),
    .COMPRESSED     (1'b0),
    .ALIGN          (1'b0),
    .MDU            (1'b0),
    .PRE_REGISTER   (1),
    .RESET_STRATEGY ("MINI"),
    .DEBUG          (1'b0),
    .WITH_CSR       (1),
    .W              (1),
    .RF_WIDTH       (2), // W=1 => W*2=2
    .RF_L2D         ($clog2((32+(1*4))*32/2)),
    .USE_STOC_ALU   (1)
  ) serv_core_inst (
    .clk         (clk_reg),
    .i_rst       (rst_reg),
    .i_timer_irq (timer_irq),

    // Example wide parallel bundling (just for demonstration):
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

    // Example MDU valid
    .o_mdu_valid (parallel_out[0])
  );

  // ---------------------------------------------------------------
  // 4. Instantiate the Parallel-to-Serial Interface
  // ---------------------------------------------------------------
  // Even though we have no external pins, we connect the module internally
  // so the logic isn't removed.
  (* DONT_TOUCH = "true" *)
  parallel_serial_if #(
    .DATA_WIDTH(DATA_WIDTH)
  ) ps_if_inst (
    .clk         (clk_reg),
    .rst         (rst_reg),
    .parallel_in (parallel_out),
    .parallel_out(parallel_in),

    // These signals remain internal, never mapped to external IO
    .ser_clk     (),
    .ser_data_out(),
    .ser_data_in (1'b0), // Tied off internally
    .ser_cs      ()
  );

endmodule