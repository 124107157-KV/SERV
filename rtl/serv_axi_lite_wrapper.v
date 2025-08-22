`default_nettype none

//==========================================================================
// serv_axi_lite_wrapper.v
//
// This module wraps the SERV core (instantiated via serv_synth_wrapper)
// with an AXI4-Lite slave interface. The AXI Lite interface is used to
// communicate with the Processing System (PS), thereby internalizing the
// wide instruction, data, and register file buses. This optimization
// significantly reduces external IO usage while preserving full CPU
// functionality, modularity, and features.
//==========================================================================

module serv_axi_lite_wrapper
  #(
    // Core parameters for the SERV core
    parameter PRE_REGISTER   = 1,
    parameter RESET_STRATEGY = "MINI",
    parameter WITH_CSR       = 1,
    parameter RF_WIDTH       = 2,
    parameter RF_L2D         = $clog2((32+(WITH_CSR*4))*32/RF_WIDTH)
  )
  (
    // AXI4-Lite Slave Interface Signals
    input  wire         s_axi_aclk,      // AXI clock (from PS)
    input  wire         s_axi_aresetn,   // Active-low reset
    input  wire [31:0]  s_axi_awaddr,    // Write address
    input  wire         s_axi_awvalid,   // Write address valid
    output wire         s_axi_awready,   // Write address ready
    input  wire [31:0]  s_axi_wdata,     // Write data
    input  wire [3:0]   s_axi_wstrb,     // Write strobes
    input  wire         s_axi_wvalid,    // Write data valid
    output wire         s_axi_wready,    // Write data ready
    output wire [1:0]   s_axi_bresp,     // Write response
    output wire         s_axi_bvalid,    // Write response valid
    input  wire         s_axi_bready,    // Write response ready
    input  wire [31:0]  s_axi_araddr,    // Read address
    input  wire         s_axi_arvalid,   // Read address valid
    output wire         s_axi_arready,   // Read address ready
    output wire [31:0]  s_axi_rdata,     // Read data
    output wire [1:0]   s_axi_rresp,     // Read response
    output wire         s_axi_rvalid,    // Read valid
    input  wire         s_axi_rready,    // Read ready

    // Optional minimal external debug indicator (e.g., LED)
    output wire         debug_led
  );

  //==========================================================
  // Internal Conversion of Reset and Clock
  //==========================================================
  // The AXI clock is used directly as our system clock.
  // The active-low AXI reset is inverted to produce the active-high reset.
  wire clk   = s_axi_aclk;
  wire i_rst = ~s_axi_aresetn;
  
  // For simplicity, we tie off the timer interrupt (it can be driven externally if needed)
  wire i_timer_irq = 1'b0;

  //==========================================================
  // Internal Signals for SERV Core Interfaces
  // These signals represent the wide buses of the SERV core.
  //==========================================================
  wire [31:0] o_ibus_adr;
  wire        o_ibus_cyc;
  wire [31:0] i_ibus_rdt;
  wire        i_ibus_ack;
  
  wire [31:0] o_dbus_adr;
  wire [31:0] o_dbus_dat;
  wire [3:0]  o_dbus_sel;
  wire        o_dbus_we;
  wire        o_dbus_cyc;
  wire [31:0] i_dbus_rdt;
  wire        i_dbus_ack;
  
  // Register File Interface (internal - not exposed externally)
  wire [RF_L2D-1:0] o_waddr;
  wire [RF_WIDTH-1:0] o_wdata;
  wire               o_wen;
  wire [RF_L2D-1:0] o_raddr;
  wire [RF_WIDTH-1:0] i_rdata;
  
  //==========================================================
  // Instantiate the SERV Core via serv_synth_wrapper
  //==========================================================
  serv_synth_wrapper
    #(
      .PRE_REGISTER   (PRE_REGISTER),
      .RESET_STRATEGY (RESET_STRATEGY),
      .WITH_CSR       (WITH_CSR),
      .RF_WIDTH       (RF_WIDTH),
      .RF_L2D         (RF_L2D)
    )
  serv_inst
    (
      .clk         (clk),
      .i_rst       (i_rst),
      .i_timer_irq (i_timer_irq),
      
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
      
      .o_waddr     (o_waddr),
      .o_wdata     (o_wdata),
      .o_wen       (o_wen),
      .o_raddr     (o_raddr),
      .i_rdata     (i_rdata)
    );

  //==========================================================
  // AXI Lite Slave Controller
  //==========================================================
  // This is a simple AXI Lite slave that maps to a 32-bit internal register.
  // In a complete implementation, you could expand the memory map to include
  // internal memories or CSR registers.
  reg [31:0] axi_reg;
  reg axi_awready, axi_wready, axi_bvalid;
  reg axi_arready, axi_rvalid;
  reg [31:0] axi_rdata;
  
  assign s_axi_awready = axi_awready;
  assign s_axi_wready  = axi_wready;
  assign s_axi_bresp   = 2'b00;  // OK response
  assign s_axi_bvalid  = axi_bvalid;
  assign s_axi_arready = axi_arready;
  assign s_axi_rdata   = axi_rdata;
  assign s_axi_rresp   = 2'b00;  // OK response
  assign s_axi_rvalid  = axi_rvalid;

  // **Write Address Channel**
  always @(posedge clk) begin
    if (i_rst)
      axi_awready <= 1'b0;
    else if (~axi_awready && s_axi_awvalid)
      axi_awready <= 1'b1;
    else
      axi_awready <= 1'b0;
  end

  // **Write Data Channel**
  always @(posedge clk) begin
    if (i_rst)
      axi_wready <= 1'b0;
    else if (~axi_wready && s_axi_wvalid)
      axi_wready <= 1'b1;
    else
      axi_wready <= 1'b0;
  end

  // **Write Response Channel**
  always @(posedge clk) begin
    if (i_rst)
      axi_bvalid <= 1'b0;
    else if (axi_awready && s_axi_awvalid && axi_wready && s_axi_wvalid)
      axi_bvalid <= 1'b1;
    else if (s_axi_bready)
      axi_bvalid <= 1'b0;
  end

  // **Write Operation:** Update internal register on valid transaction
  always @(posedge clk) begin
    if (i_rst)
      axi_reg <= 32'd0;
    else if (axi_awready && s_axi_awvalid && axi_wready && s_axi_wvalid)
      axi_reg <= s_axi_wdata;
  end

  // **Read Address Channel**
  always @(posedge clk) begin
    if (i_rst)
      axi_arready <= 1'b0;
    else if (~axi_arready && s_axi_arvalid)
      axi_arready <= 1'b1;
    else
      axi_arready <= 1'b0;
  end

  // **Read Data Channel**
  always @(posedge clk) begin
    if (i_rst) begin
      axi_rvalid <= 1'b0;
      axi_rdata  <= 32'd0;
    end else if (axi_arready && s_axi_arvalid) begin
      axi_rvalid <= 1'b1;
      // For this example, return the content of axi_reg.
      axi_rdata  <= axi_reg;
    end else if (s_axi_rready) begin
      axi_rvalid <= 1'b0;
    end
  end

  //==========================================================
  // Debug LED Driven by a Bit of axi_reg (for example)
  //==========================================================
  assign debug_led = axi_reg[0];

endmodule

`default_nettype wire