`default_nettype none

module serv_rf_axi_wrapper
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
    parameter USE_STOC_ALU   = 1
  )
  (
    // AXI4-Lite Slave Interface
    input  wire         s_axi_aclk,
    input  wire         s_axi_aresetn,
    input  wire [31:0]  s_axi_awaddr,
    input  wire         s_axi_awvalid,
    output wire         s_axi_awready,
    input  wire [31:0]  s_axi_wdata,
    input  wire [3:0]   s_axi_wstrb,
    input  wire         s_axi_wvalid,
    output wire         s_axi_wready,
    output wire [1:0]   s_axi_bresp,
    output wire         s_axi_bvalid,
    input  wire         s_axi_bready,
    input  wire [31:0]  s_axi_araddr,
    input  wire         s_axi_arvalid,
    output wire         s_axi_arready,
    output wire [31:0]  s_axi_rdata,
    output wire [1:0]   s_axi_rresp,
    output wire         s_axi_rvalid,
    input  wire         s_axi_rready,
    
    // Debug output
    output wire         debug_led
  );

  // Clock and reset signals
  wire clk = s_axi_aclk;
  wire i_rst = ~s_axi_aresetn;
  wire i_timer_irq = 1'b0;

  // Internal signals
  wire [31:0] int_ibus_adr, int_dbus_adr, int_dbus_dat, int_ibus_rdt, int_dbus_rdt;
  wire int_ibus_cyc, int_dbus_cyc, int_dbus_we, int_ibus_ack, int_dbus_ack;
  wire [3:0]  int_dbus_sel;
  wire int_mdu_valid;

  //==========================================================
  // Instantiate SERV Core (serv_rf_top)
  //==========================================================
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
  serv_core
    (
     .clk         (clk),
     .i_rst       (i_rst),
     .i_timer_irq (i_timer_irq),

     .o_ibus_adr  (int_ibus_adr),
     .o_ibus_cyc  (int_ibus_cyc),
     .i_ibus_rdt  (int_ibus_rdt),
     .i_ibus_ack  (int_ibus_ack),

     .o_dbus_adr  (int_dbus_adr),
     .o_dbus_dat  (int_dbus_dat),
     .o_dbus_sel  (int_dbus_sel),
     .o_dbus_we   (int_dbus_we),
     .o_dbus_cyc  (int_dbus_cyc),
     .i_dbus_rdt  (int_dbus_rdt),
     .i_dbus_ack  (int_dbus_ack),

     .o_mdu_valid (int_mdu_valid)
    );

  //==========================================================
  // AXI4-Lite Slave Controller
  //==========================================================
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

  always @(posedge clk) begin
    if (i_rst) axi_awready <= 1'b0;
    else if (~axi_awready && s_axi_awvalid) axi_awready <= 1'b1;
    else axi_awready <= 1'b0;
  end

  always @(posedge clk) begin
    if (i_rst) axi_wready <= 1'b0;
    else if (~axi_wready && s_axi_wvalid) axi_wready <= 1'b1;
    else axi_wready <= 1'b0;
  end

  always @(posedge clk) begin
    if (i_rst) axi_bvalid <= 1'b0;
    else if (axi_awready && s_axi_awvalid && axi_wready && s_axi_wvalid) axi_bvalid <= 1'b1;
    else if (s_axi_bready) axi_bvalid <= 1'b0;
  end

  always @(posedge clk) begin
    if (i_rst) axi_rvalid <= 1'b0;
    else if (axi_arready && s_axi_arvalid) begin
      axi_rvalid <= 1'b1;
      axi_rdata  <= axi_reg;
    end else if (s_axi_rready) axi_rvalid <= 1'b0;
  end

  assign debug_led = axi_reg[0];

endmodule

`default_nettype wire
