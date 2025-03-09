`default_nettype none
//----------------------------------------------------------------------
// To enable IO bundling (to reduce external pin count), uncomment the following define.
// In production builds you want to define this, while for debugging you may leave it undefined.
//----------------------------------------------------------------------
//`define BUNDLE_MEM_IF_IO

module serv_mem_if
  #(
    parameter [0:0] WITH_CSR = 1,
    parameter       W = 1,
    parameter       B = W-1
  )
  (
   input wire i_clk,
`ifdef BUNDLE_MEM_IF_IO
   // Bundle state signals into one 4-bit bus:
   //   [3:2] = i_bytecnt, [1:0] = i_lsb
   input wire [3:0] i_state,
`else
   // State signals (unbundled)
   input wire [1:0] i_bytecnt,
   input wire [1:0] i_lsb,
`endif
   output wire o_byte_valid,
   output wire o_misalign,
   // Control signals
   input wire i_signed,
   input wire i_word,
   input wire i_half,
   // MDU control
   input wire i_mdu_op,
   // Data input
   input wire [B:0] i_bufreg2_q,
   // Data output
   output wire [B:0] o_rd,
   // External interface: Byte enable for the data bus
   output wire [3:0] o_wb_sel
   );

`ifdef BUNDLE_MEM_IF_IO
   // If state signals are bundled, create aliases:
   wire [1:0] i_bytecnt = i_state[3:2];
   wire [1:0] i_lsb     = i_state[1:0];
`endif

   //----------------------------------------------------------------------------
   // Declare internal signal "signbit" before its use.
   reg signbit;
   
   //----------------------------------------------------------------------------
   // Compute Byte Validity
   // The expression below is a synthesis-friendly way to generate a "byte valid"
   // signal based on the combination of i_lsb and i_bytecnt. (This achieves the
   // condition "shift when i_lsb + i_bytecnt < 4".)
   assign o_byte_valid =
       (!i_lsb[0] & !i_lsb[1])         |
       (!i_bytecnt[0] & !i_bytecnt[1]) |
       (!i_bytecnt[1] & !i_lsb[1])     |
       (!i_bytecnt[1] & !i_lsb[0])     |
       (!i_bytecnt[0] & !i_lsb[1]);

   //----------------------------------------------------------------------------
   // Determine Data Validity
   // Data from i_bufreg2_q is considered valid if any of the following is true:
   //   - An MDU operation is active,
   //   - A word operation is active,
   //   - The byte count is zero (aligned),
   //   - It is a half-word operation and the MSB of the byte count is 0.
   wire dat_valid =
       i_mdu_op |
       i_word |
       (i_bytecnt == 2'b00) |
       (i_half & !i_bytecnt[1]);

   //----------------------------------------------------------------------------
   // Data Output Selection (o_rd)
   // If the data is valid, pass through i_bufreg2_q.
   // Otherwise, output a sign-extended version based on i_signed and signbit.
   assign o_rd = dat_valid ? i_bufreg2_q : {W{i_signed & signbit}};

   //----------------------------------------------------------------------------
   // Write Byte Selection (o_wb_sel)
   // Each bit of o_wb_sel determines which byte lane is enabled.
   // They are derived from i_lsb and the operation type.
   assign o_wb_sel[3] = (i_lsb == 2'b11) | i_word | (i_half & i_lsb[1]);
   assign o_wb_sel[2] = (i_lsb == 2'b10) | i_word;
   assign o_wb_sel[1] = (i_lsb == 2'b01) | i_word | (i_half & !i_lsb[1]);
   assign o_wb_sel[0] = (i_lsb == 2'b00);

   //----------------------------------------------------------------------------
   // Sign Extension: Capture the most-significant bit of i_bufreg2_q
   // when data is valid. This bit is later used to sign-extend o_rd when data
   // is not valid.
   always @(posedge i_clk) begin
      if (dat_valid)
         signbit <= i_bufreg2_q[B];
   end

   //----------------------------------------------------------------------------
   // Misalignment Detection (o_misalign)
   // When CSR support is enabled (WITH_CSR==1), assert o_misalign if the address
   // (i_lsb) and operation type indicate misalignment.
   assign o_misalign = WITH_CSR & ((i_lsb[0] & (i_word | i_half)) | (i_lsb[1] & i_word));

endmodule
`default_nettype wire