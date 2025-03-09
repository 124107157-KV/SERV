`default_nettype none
//----------------------------------------------------------------------
// To enable bundled memory interface inputs (to reduce external IO pins),
// uncomment the following define. In production builds (or when IO is tight),
// you may enable this option.
//----------------------------------------------------------------------
//`define BUNDLE_MEM_IF

module serv_rf_ram
  #(parameter width    = 0,
    parameter csr_regs = 4,
    parameter depth    = 32*(32+csr_regs)/width)
   (input  wire i_clk,
`ifdef BUNDLE_MEM_IF
    // When bundling is enabled, the following single bus replaces:
    //   i_waddr, i_wdata, i_wen, i_raddr, and i_ren.
    // The bundled bus width is calculated as follows:
    //   AW = $clog2(depth)
    //   Bundle width = (AW + width + 1)   --> for the write interface
    //                + (AW + 1)         --> for the read interface
    //                = 2*AW + width + 2.
    input  wire [BUNDLE_WIDTH-1:0] i_mem_if,
`else
    input  wire [$clog2(depth)-1:0] i_waddr,
    input  wire [width-1:0]         i_wdata,
    input  wire                     i_wen,
    input  wire [$clog2(depth)-1:0] i_raddr,
    input  wire                     i_ren,
`endif
    output wire [width-1:0]         o_rdata);

`ifdef BUNDLE_MEM_IF
   // Calculate the address width.
   localparam AW = $clog2(depth);
   // Total bundled width = 2*AW + width + 2.
   localparam BUNDLE_WIDTH = 2*AW + width + 2;
   // Extract the bundled signals:
   // The bundled bus is organized (from MSB down to LSB) as follows:
   //   [BUNDLE_WIDTH-1   : BUNDLE_WIDTH-AW]         --> write address (AW bits)
   //   [BUNDLE_WIDTH-AW-1: BUNDLE_WIDTH-AW-width]      --> write data (width bits)
   //   [BUNDLE_WIDTH-AW-width-1]                        --> write enable (1 bit)
   //   [BUNDLE_WIDTH-AW-width-2: BUNDLE_WIDTH-AW-width-1-AW] --> read address (AW bits)
   //   [0]                                             --> read enable (1 bit)
   wire [AW-1:0]     bundled_waddr;
   wire [width-1:0]  bundled_wdata;
   wire              bundled_wen;
   wire [AW-1:0]     bundled_raddr;
   wire              bundled_ren;
   assign bundled_waddr = i_mem_if[BUNDLE_WIDTH-1 -: AW];
   assign bundled_wdata = i_mem_if[BUNDLE_WIDTH-AW-1 -: width];
   assign bundled_wen   = i_mem_if[BUNDLE_WIDTH-AW-width-1];
   assign bundled_raddr = i_mem_if[BUNDLE_WIDTH-AW-width-2 -: AW];
   assign bundled_ren   = i_mem_if[0];
   
   // Create internal aliases for use in the logic.
   wire [$clog2(depth)-1:0] actual_waddr = bundled_waddr;
   wire [width-1:0]         actual_wdata = bundled_wdata;
   wire                     actual_wen   = bundled_wen;
   wire [$clog2(depth)-1:0] actual_raddr = bundled_raddr;
   wire                     actual_ren   = bundled_ren;
`else
   // If bundling is disabled, use the original ports.
   wire [$clog2(depth)-1:0] actual_waddr = i_waddr;
   wire [width-1:0]         actual_wdata = i_wdata;
   wire                     actual_wen   = i_wen;
   wire [$clog2(depth)-1:0] actual_raddr = i_raddr;
   wire                     actual_ren   = i_ren;
`endif

   // Internal memory array declaration.
   reg [width-1:0] memory [0:depth-1];
   reg [width-1:0] rdata;

   // Synchronous write and read operations.
   always @(posedge i_clk) begin
      if (actual_wen)
         memory[actual_waddr] <= actual_wdata;
      rdata <= actual_ren ? memory[actual_raddr] : {width{1'bx}};
   end

   /* 
    * Enforce RISC-V convention: Reads from register x0 must return 0.
    * The following logic checks that the portion of the read address
    * corresponding to the register index is zero and gates the output.
    * The slice of the read address checked depends on the word width.
    */
   reg regzero;
   always @(posedge i_clk)
     regzero <= !(|actual_raddr[$clog2(depth)-1:5-$clog2(width)]);

   assign o_rdata = rdata & ~{width{regzero}};

`ifdef SERV_CLEAR_RAM
   // Optionally initialize (clear) the memory on startup.
   integer i;
   initial begin
     for (i = 0; i < depth; i = i + 1)
       memory[i] = {width{1'd0}};
   end
`endif

endmodule
`default_nettype wire
