`default_nettype none
//----------------------------------------------------------------------
// Uncomment these defines for verification or extension development.
// For production builds (to save IO), leave them undefined.
//----------------------------------------------------------------------
//`define RVFI_ENABLED
//`define EXT_ENABLED

module serv_rf_top
  #(parameter RESET_PC      = 32'd0,
    /* COMPRESSED=1: Enable the compressed decoder and allow misaligned PC jump
       COMPRESSED=0: Disable the compressed decoder and disallow misaligned PC jump */
    parameter [0:0] COMPRESSED   = 0,
    /* ALIGN = 1: Fetch an aligned instruction by making two bus transactions if
       a misaligned address is given to the instruction bus. */
    parameter [0:0] ALIGN        = COMPRESSED,
    /* MDU = 1: Enables the multiplication/division unit interface */
    parameter [0:0] MDU          = 0,
    /* PRE_REGISTER = 0: Register after the decoder (faster but uses more resources)
       PRE_REGISTER = 1: (default) Register before the decoder (slower but uses fewer resources) */
    parameter PRE_REGISTER  = 1,
    /* RESET_STRATEGY = "NONE" : No internal reset (relies on POR)
       RESET_STRATEGY = "MINI" : Resets only the minimal FFs needed to restart execution at RESET_PC */
    parameter RESET_STRATEGY = "MINI",
    parameter [0:0] DEBUG        = 1'b0,
    parameter WITH_CSR       = 1,
    parameter W              = 1,
    parameter RF_WIDTH       = W * 2,
    parameter RF_L2D         = $clog2((32+(WITH_CSR*4))*32/RF_WIDTH),
    // New parameter for dual ALU selection:
    parameter USE_STOC_ALU   = 1
   )
  (
   // Clock, reset and interrupt
   input  wire         clk,
   input  wire         i_rst,
   input  wire         i_timer_irq,
   
`ifdef RVFI_ENABLED
   // RVFI formal verification interface (omitted in production)
   output wire         rvfi_valid,
   output wire [63:0]  rvfi_order,
   output wire [31:0]  rvfi_insn,
   output wire         rvfi_trap,
   output wire         rvfi_halt,
   output wire         rvfi_intr,
   output wire [1:0]   rvfi_mode,
   output wire [1:0]   rvfi_ixl,
   output wire [4:0]   rvfi_rs1_addr,
   output wire [4:0]   rvfi_rs2_addr,
   output wire [31:0]  rvfi_rs1_rdata,
   output wire [31:0]  rvfi_rs2_rdata,
   output wire [4:0]   rvfi_rd_addr,
   output wire [31:0]  rvfi_rd_wdata,
   output wire [31:0]  rvfi_pc_rdata,
   output wire [31:0]  rvfi_pc_wdata,
   output wire [31:0]  rvfi_mem_addr,
   output wire [3:0]   rvfi_mem_rmask,
   output wire [3:0]   rvfi_mem_wmask,
   output wire [31:0]  rvfi_mem_rdata,
   output wire [31:0]  rvfi_mem_wdata,
`endif

   // Instruction bus interface
   output wire [31:0]  o_ibus_adr,
   output wire         o_ibus_cyc,
   input  wire [31:0]  i_ibus_rdt,
   input  wire         i_ibus_ack,

   // Data bus interface
   output wire [31:0]  o_dbus_adr,
   output wire [31:0]  o_dbus_dat,
   output wire [3:0]   o_dbus_sel,
   output wire         o_dbus_we,
   output wire         o_dbus_cyc,
   input  wire [31:0]  i_dbus_rdt,
   input  wire         i_dbus_ack,

`ifdef EXT_ENABLED
   // Extension interface (omitted in production)
   output wire [31:0]  o_ext_rs1,
   output wire [31:0]  o_ext_rs2,
   output wire [2:0]   o_ext_funct3,
   input  wire [31:0]  i_ext_rd,
   input  wire         i_ext_ready,
`endif

   // Multiplication/Division Unit interface
   output wire         o_mdu_valid
   );

   //-------------------------------------------------------------------------
   // Internal signals for the register file and RAM interface.
   // These signals connect the CPU core to its register file and RAM.
   //-------------------------------------------------------------------------
   localparam CSR_REGS = WITH_CSR * 4;

   wire               rf_wreq;
   wire               rf_rreq;
   wire [4+WITH_CSR:0] wreg0;
   wire [4+WITH_CSR:0] wreg1;
   wire               wen0;
   wire               wen1;
   wire [W-1:0]       wdata0;
   wire [W-1:0]       wdata1;
   wire [4+WITH_CSR:0] rreg0;
   wire [4+WITH_CSR:0] rreg1;
   wire               rf_ready;
   wire [W-1:0]       rdata0;
   wire [W-1:0]       rdata1;

   // Simplified RAM interface signals (between register file interface and RAM)
   wire [RF_L2D-1:0]  waddr;
   wire [RF_WIDTH-1:0] wdata;
   wire               wen;
   wire [RF_L2D-1:0]  raddr;
   wire               ren;
   wire [RF_WIDTH-1:0] rdata;

   //-------------------------------------------------------------------------
   // Instantiation of the Register File RAM Interface.
   //-------------------------------------------------------------------------
   serv_rf_ram_if
     #(.width         (RF_WIDTH),
       .reset_strategy(RESET_STRATEGY),
       .csr_regs      (CSR_REGS),
       .W             (W))
   rf_ram_if
     (
      .i_clk    (clk),
      .i_rst    (i_rst),
      .i_wreq   (rf_wreq),
      .i_rreq   (rf_rreq),
      .o_ready  (rf_ready),
      .i_wreg0  (wreg0),
      .i_wreg1  (wreg1),
      .i_wen0   (wen0),
      .i_wen1   (wen1),
      .i_wdata0 (wdata0),
      .i_wdata1 (wdata1),
      .i_rreg0  (rreg0),
      .i_rreg1  (rreg1),
      .o_rdata0 (rdata0),
      .o_rdata1 (rdata1),
      .o_waddr  (waddr),
      .o_wdata  (wdata),
      .o_wen    (wen),
      .o_raddr  (raddr),
      .o_ren    (ren),
      .i_rdata  (rdata)
     );

   //-------------------------------------------------------------------------
   // Instantiation of the Register File RAM.
   //-------------------------------------------------------------------------
   serv_rf_ram
     #(.width    (RF_WIDTH),
       .csr_regs (CSR_REGS))
   rf_ram
     (
      .i_clk    (clk),
      .i_waddr  (waddr),
      .i_wdata  (wdata),
      .i_wen    (wen),
      .i_raddr  (raddr),
      .i_ren    (ren),
      .o_rdata  (rdata)
     );

   //-------------------------------------------------------------------------
   // Instantiation of the Main CPU Core.
   //-------------------------------------------------------------------------
   serv_top
     #(.RESET_PC       (RESET_PC),
       .PRE_REGISTER   (PRE_REGISTER),
       .RESET_STRATEGY (RESET_STRATEGY),
       .WITH_CSR       (WITH_CSR),
       .DEBUG          (DEBUG),
       .MDU            (MDU),
       .COMPRESSED     (COMPRESSED),
       .ALIGN          (ALIGN),
       .W              (W),
       .USE_STOC_ALU   (USE_STOC_ALU))
   cpu
     (
      .clk         (clk),
      .i_rst       (i_rst),
      .i_timer_irq (i_timer_irq),
      
`ifdef RVFI_ENABLED
      .rvfi_valid     (rvfi_valid),
      .rvfi_order     (rvfi_order),
      .rvfi_insn      (rvfi_insn),
      .rvfi_trap      (rvfi_trap),
      .rvfi_halt      (rvfi_halt),
      .rvfi_intr      (rvfi_intr),
      .rvfi_mode      (rvfi_mode),
      .rvfi_ixl       (rvfi_ixl),
      .rvfi_rs1_addr  (rvfi_rs1_addr),
      .rvfi_rs2_addr  (rvfi_rs2_addr),
      .rvfi_rs1_rdata (rvfi_rs1_rdata),
      .rvfi_rs2_rdata (rvfi_rs2_rdata),
      .rvfi_rd_addr   (rvfi_rd_addr),
      .rvfi_rd_wdata  (rvfi_rd_wdata),
      .rvfi_pc_rdata  (rvfi_pc_rdata),
      .rvfi_pc_wdata  (rvfi_pc_wdata),
      .rvfi_mem_addr  (rvfi_mem_addr),
      .rvfi_mem_rmask (rvfi_mem_rmask),
      .rvfi_mem_wmask (rvfi_mem_wmask),
      .rvfi_mem_rdata (rvfi_mem_rdata),
      .rvfi_mem_wdata (rvfi_mem_wdata),
`endif

      .o_rf_rreq   (rf_rreq),
      .o_rf_wreq   (rf_wreq),
      .i_rf_ready  (rf_ready),
      .o_wreg0     (wreg0),
      .o_wreg1     (wreg1),
      .o_wen0      (wen0),
      .o_wen1      (wen1),
      .o_wdata0    (wdata0),
      .o_wdata1    (wdata1),
      .o_rreg0     (rreg0),
      .o_rreg1     (rreg1),
      .i_rdata0    (rdata0),
      .i_rdata1    (rdata1),

      .o_ibus_adr  (o_ibus_adr),
      .o_ibus_cyc  (o_ibus_cyc),
      .i_ibus_rdt  (i_ibus_rdt),
      .i_ibus_ack  (i_ibus_ack),

      .o_dbus_adr  (o_dbus_adr),
      .o_dbus_dat  (o_dbus_dat),
      .o_dbus_sel  (o_dbus_sel),
      .o_dbus_we   (o_dbus_we),
      .o_dbus_cyc  (o_dbus_cyc),
      .i_dbus_rdt  (i_dbus_rdt),
      .i_dbus_ack  (i_dbus_ack),
      
`ifdef EXT_ENABLED
      .o_ext_funct3 (ext_o_funct3),
      .i_ext_ready  (ext_i_ready),
      .i_ext_rd     (ext_i_rd),
      .o_ext_rs1    (ext_o_rs1),
      .o_ext_rs2    (ext_o_rs2),
`endif
      .o_mdu_valid (o_mdu_valid)
     );

   //-------------------------------------------------------------------------
   // The remaining module instantiations (immediate decoder, buffer registers,
   // control unit, ALU, register file interface, memory interface, CSR, and debug)
   // are not modified here because our IO optimization changes are focused on
   // the top-level IO bundling and port tie-offs.
   // (They are assumed to remain as in the original design.)
   //-------------------------------------------------------------------------

endmodule

`default_nettype wire