`default_nettype none
//----------------------------------------------------------------------
// Define BUNDLE_CSR_IO to bundle external IO signals (for production builds).
// Uncomment the following line in production to reduce IO count.
//----------------------------------------------------------------------
//`define BUNDLE_CSR_IO

module serv_csr
  #(
    parameter RESET_STRATEGY = "MINI",
    parameter W = 1,
    parameter B = W-1
  )
  (
   input wire        i_clk,
   input wire        i_rst,
   
`ifdef BUNDLE_CSR_IO
   // Bundle many state signals into one 10-bit bus.
   // Mapping (example):
   // [9]   : i_trig_irq
   // [8]   : i_en
   // [7]   : i_cnt0to3
   // [6]   : i_cnt3
   // [5]   : i_cnt7
   // [4]   : i_cnt11
   // [3]   : i_cnt12
   // [2]   : i_cnt_done
   // [1]   : i_mem_op
   // [0]   : i_mtip
   input wire [9:0] state_bus,
   // Trap signal remains unbundled (critical)
   input wire       i_trap,
   // Bundle control signals into one 6-bit bus.
   // Mapping:
   // [5] : i_e_op
   // [4] : i_ebreak
   // [3] : i_mem_cmd
   // [2] : i_mstatus_en
   // [1] : i_mie_en
   // [0] : i_mcause_en
   input wire [5:0] ctrl_bus,
   // The following control signals remain unbundled.
   input wire [1:0] i_csr_source,
   input wire       i_mret,
   input wire       i_csr_d_sel,
`else
   // Unbundled state signals.
   input wire       i_trig_irq,
   input wire       i_en,
   input wire       i_cnt0to3,
   input wire       i_cnt3,
   input wire       i_cnt7,
   input wire       i_cnt11,
   input wire       i_cnt12,
   input wire       i_cnt_done,
   input wire       i_mem_op,
   input wire       i_mtip,
   // Trap signal.
   input wire       i_trap,
   // Unbundled control signals.
   input wire       i_e_op,
   input wire       i_ebreak,
   input wire       i_mem_cmd,
   input wire       i_mstatus_en,
   input wire       i_mie_en,
   input wire       i_mcause_en,
   input wire [1:0] i_csr_source,
   input wire       i_mret,
   input wire       i_csr_d_sel,
`endif

   // Data signals (always unbundled)
   input wire [B:0] i_rf_csr_out,
   input wire [B:0] i_csr_imm,
   input wire [B:0] i_rs1,
   
`ifdef BUNDLE_CSR_IO
   // Bundle the CSR-related outputs into one bus.
   // Bus mapping (example for total width = 2*W+1):
   // [2*W-1:W] : o_csr_in, [W-1:0] : o_q, LSB: o_new_irq
   output wire [2*W:0] csr_bus
`else
   output wire [B:0] o_csr_in,
   output wire [B:0] o_q,
   output reg        o_new_irq
`endif
   );

`ifdef BUNDLE_CSR_IO
   // Extract state signals from the bundled state_bus.
   wire i_trig_irq = state_bus[9];
   wire i_en       = state_bus[8];
   wire i_cnt0to3  = state_bus[7];
   wire i_cnt3     = state_bus[6];
   wire i_cnt7     = state_bus[5];
   wire i_cnt11    = state_bus[4];
   wire i_cnt12    = state_bus[3];
   wire i_cnt_done = state_bus[2];
   wire i_mem_op   = state_bus[1];
   wire i_mtip     = state_bus[0];
`endif

   //-------------------------------------------------------------------------
   // Local parameters for CSR source selection.
   localparam [1:0]
     CSR_SOURCE_CSR = 2'b00,
     CSR_SOURCE_EXT = 2'b01,
     CSR_SOURCE_SET = 2'b10,
     CSR_SOURCE_CLR = 2'b11;

   //-------------------------------------------------------------------------
   // Internal registers for mstatus and interrupt enables.
   reg mstatus_mie;
   reg mstatus_mpie;
   reg mie_mtie;

   // Registers for the exception cause.
   reg mcause31;
   reg [3:0] mcause3_0;
   wire [B:0] mcause;

   // Intermediate wires for the new CSR value.
   wire [B:0] csr_in;
   wire [B:0] csr_out;

   // Register to latch the timer interrupt.
   reg timer_irq_r;

   //-------------------------------------------------------------------------
   // Compute the data used for CSR updates.
   // If i_csr_d_sel is true, select the CSR immediate; otherwise, select RS1.
   wire [B:0] d = i_csr_d_sel ? i_csr_imm : i_rs1;

   //-------------------------------------------------------------------------
   // Compute the new CSR value (csr_in) based on the selected source.
   assign csr_in = (i_csr_source == CSR_SOURCE_EXT) ? d :
                   (i_csr_source == CSR_SOURCE_SET) ? csr_out | d :
                   (i_csr_source == CSR_SOURCE_CLR) ? csr_out & ~d :
                   (i_csr_source == CSR_SOURCE_CSR) ? csr_out :
                   {W{1'bx}};

   //-------------------------------------------------------------------------
   // Compute mstatus from internal registers.
   wire [B:0] mstatus;
   generate
      if (W==1) begin : gen_mstatus_w1
         assign mstatus = ((mstatus_mie & i_cnt3) | (i_cnt11 | i_cnt12));
      end else if (W==4) begin : gen_mstatus_w4
         assign mstatus = { i_cnt11 | (mstatus_mie & i_cnt3), 2'b00, i_cnt12 };
      end
   endgenerate

   //-------------------------------------------------------------------------
   // Combine the mstatus, RF CSR output, and mcause to form the current CSR.
   assign csr_out = ({W{i_mstatus_en & i_en}} & mstatus) |
                    i_rf_csr_out |
                    ({W{i_mcause_en & i_en}} & mcause);

`ifndef BUNDLE_CSR_IO
   // When not bundling, drive o_q with csr_out.
   assign o_q = csr_out;
`endif

   //-------------------------------------------------------------------------
   // Compute the timer interrupt signal.
   wire timer_irq = i_mtip & mstatus_mie & mie_mtie;

   //-------------------------------------------------------------------------
   // Compute mcause.
   assign mcause = i_cnt0to3 ? mcause3_0[B:0] :
                   i_cnt_done ? {mcause31, {B{1'b0}}} :
                   {W{1'b0}};

   //-------------------------------------------------------------------------
   // o_csr_in is driven with the newly computed CSR value.
   assign o_csr_in = csr_in;

   //-------------------------------------------------------------------------
   // Sequential logic to update internal registers and generate new IRQ.
   always @(posedge i_clk) begin
      if (i_trig_irq) begin
         timer_irq_r <= timer_irq;
         o_new_irq <= timer_irq & !timer_irq_r;
      end

      if (i_mie_en & i_cnt7)
         mie_mtie <= csr_in[B];

      // Update mstatus_mie.
      if ((i_trap & i_cnt_done) | (i_mstatus_en & i_cnt3 & i_en) | i_mret)
         mstatus_mie <= !i_trap & (i_mret ? mstatus_mpie : csr_in[B]);

      // Latch mstatus_mie into mstatus_mpie on trap.
      if (i_trap & i_cnt_done)
         mstatus_mpie <= mstatus_mie;

      // Update mcause3_0 (lower 4 bits of mcause) during CSR access or trap.
      if ((i_mcause_en & i_en & i_cnt0to3) | (i_trap & i_cnt_done)) begin
         mcause3_0[3] <= (i_e_op & !i_ebreak) | (!i_trap & csr_in[B]);
         mcause3_0[2] <= o_new_irq | i_mem_op | (!i_trap & ((W == 1) ? mcause3_0[3] : csr_in[(W == 1) ? 0 : 2]));
         mcause3_0[1] <= o_new_irq | i_e_op | (i_mem_op & i_mem_cmd) | (!i_trap & ((W == 1) ? mcause3_0[2] : csr_in[(W == 1) ? 0 : 1]));
         mcause3_0[0] <= o_new_irq | i_e_op | (!i_trap & ((W == 1) ? mcause3_0[1] : csr_in[0]));
      end

      // Update mcause31 (the MSB of mcause).
      if ((i_mcause_en & i_cnt_done) | i_trap)
         mcause31 <= i_trap ? o_new_irq : csr_in[B];

      if (i_rst)
         if (RESET_STRATEGY != "NONE") begin
            o_new_irq <= 1'b0;
            mie_mtie <= 1'b0;
         end
   end

`ifdef BUNDLE_CSR_IO
   // In the bundled interface, pack outputs into a single bus.
   // The bus is [2*W:0] bits wide, where:
   //   - Bits [2*W-1:W] correspond to o_csr_in,
   //   - Bits [W-1:0] correspond to o_q,
   //   - Bit [0] corresponds to o_new_irq.
   // (Downstream logic must extract these fields as needed.)
   wire [2*W:0] bundled_csr_bus;
   assign bundled_csr_bus = { o_csr_in, o_q, o_new_irq };
`endif

endmodule
`default_nettype wire