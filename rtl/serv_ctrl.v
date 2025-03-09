`default_nettype none

//----------------------------------------------------------------------
// Uncomment the following defines to enable IO bundling.
// For production builds (to save IO), define these; for debugging, leave undefined.
//----------------------------------------------------------------------
//`define BUNDLE_CTRL_INPUTS
//`define BUNDLE_CTRL_OUTPUTS

module serv_ctrl
  #(parameter RESET_STRATEGY = "MINI",
    parameter RESET_PC = 32'd0,
    parameter WITH_CSR = 1,
    parameter W = 1,
    parameter B = W-1
  )
  (
   input  wire clk,
   input  wire i_rst,
`ifdef BUNDLE_CTRL_INPUTS
   // Bundle state signals into one 5-bit bus:
   //   state_in[4] = i_cnt12to31, [3] = i_cnt0, [2] = i_cnt1, [1] = i_cnt2, [0] = i_pc_en
   input  wire [4:0] state_in,
   // Bundle control signals into one 6-bit bus:
   //   ctrl_in[5] = i_jump, [4] = i_jal_or_jalr, [3] = i_utype, [2] = i_pc_rel, [1] = i_trap, [0] = i_iscomp
   input  wire [5:0] ctrl_in,
`else
   // State signals
   input  wire i_pc_en,
   input  wire i_cnt12to31,
   input  wire i_cnt0,
   input  wire i_cnt1,
   input  wire i_cnt2,
   // Control signals
   input  wire i_jump,
   input  wire i_jal_or_jalr,
   input  wire i_utype,
   input  wire i_pc_rel,
   input  wire i_trap,
   input  wire i_iscomp,
`endif
   // Data signals
   input  wire [B:0] i_imm,
   input  wire [B:0] i_buf,
   input  wire [B:0] i_csr_pc,
`ifdef BUNDLE_CTRL_OUTPUTS
   // Bundle o_rd and o_bad_pc into a single bus.
   // Let the bus width be 2*(B+1) bits: upper half is o_rd and lower half is o_bad_pc.
   output wire [2*(B+1)-1:0] o_ctrl_data,
`else
   output wire [B:0] o_rd,
   output wire [B:0] o_bad_pc,
`endif
   // External output for instruction bus address (remains separate)
   output reg [31:0] o_ibus_adr
   );

   //-------------------------------------------------------------------------
   // If inputs are bundled, alias them to the original signal names.
`ifdef BUNDLE_CTRL_INPUTS
   // State inputs:
   wire i_pc_en    = state_in[0];
   wire i_cnt2     = state_in[1];
   wire i_cnt1     = state_in[2];
   wire i_cnt0     = state_in[3];
   wire i_cnt12to31 = state_in[4];
   // Control inputs:
   wire i_jump         = ctrl_in[5];
   wire i_jal_or_jalr  = ctrl_in[4];
   wire i_utype        = ctrl_in[3];
   wire i_pc_rel       = ctrl_in[2];
   wire i_trap         = ctrl_in[1];
   wire i_iscomp       = ctrl_in[0];
`endif

   //-------------------------------------------------------------------------
   // Internal signals for PC arithmetic and offset computation
   //-------------------------------------------------------------------------
   wire [B:0] pc_plus_4;
   wire       pc_plus_4_cy;
   reg        pc_plus_4_cy_r;
   wire [B:0] pc_plus_4_cy_r_w;
   wire [B:0] pc_plus_offset;
   wire       pc_plus_offset_cy;
   reg        pc_plus_offset_cy_r;
   wire [B:0] pc_plus_offset_cy_r_w;
   wire [B:0] pc_plus_offset_aligned;
   wire [B:0] plus_4;

   // The current PC is taken from the lower (B+1) bits of o_ibus_adr.
   wire [B:0] pc = o_ibus_adr[B:0];

   wire [B:0] new_pc;

   wire [B:0] offset_a;
   wire [B:0] offset_b;

   /* If i_iscomp=1, increment PC by 2; otherwise increment by 4 */
   generate
      if (W == 1) begin : gen_plus_4_w_eq_1
         assign plus_4 = i_iscomp ? i_cnt1 : i_cnt2;
      end else if (W == 4) begin : gen_plus_4_w_eq_4
         assign plus_4 = (i_cnt0 | i_cnt1) ? (i_iscomp ? 2 : 4) : 0;
      end
   endgenerate

   // The "bad PC" output is the aligned offset.
   assign o_bad_pc = pc_plus_offset_aligned;

   // PC+4 computation: add the current PC, plus_4, and an incoming carry.
   assign {pc_plus_4_cy, pc_plus_4} = pc + plus_4 + pc_plus_4_cy_r_w;

   generate
      if (|WITH_CSR) begin : gen_csr
         if (W == 1) begin : gen_new_pc_w_eq_1
            assign new_pc = i_trap ? (i_csr_pc & !(i_cnt0 || i_cnt1))
                                   : i_jump ? pc_plus_offset_aligned
                                            : pc_plus_4;
         end else if (W == 4) begin : gen_new_pc_w_eq_4
            assign new_pc = i_trap ? (i_csr_pc & ((i_cnt0 || i_cnt1) ? 4'b1100 : 4'b1111))
                                   : i_jump ? pc_plus_offset_aligned
                                            : pc_plus_4;
         end
      end else begin : gen_no_csr
         assign new_pc = i_jump ? pc_plus_offset_aligned : pc_plus_4;
      end
   endgenerate

   // o_rd is computed based on the type of instruction.
   assign o_rd = ({W{i_utype}} & pc_plus_offset_aligned) | (pc_plus_4 & {W{i_jal_or_jalr}});

   // Compute the offset to add to the PC.
   assign offset_a = {W{i_pc_rel}} & pc;
   assign offset_b = i_utype ? (i_imm & {W{i_cnt12to31}}) : i_buf;
   assign {pc_plus_offset_cy, pc_plus_offset} = offset_a + offset_b + pc_plus_offset_cy_r_w;

   generate
      if (W > 1) begin : gen_w_gt_1
         assign pc_plus_offset_aligned[B:1] = pc_plus_offset[B:1];
         assign pc_plus_offset_cy_r_w[B:1] = {B{1'b0}};
         assign pc_plus_4_cy_r_w[B:1] = {B{1'b0}};
      end
   endgenerate

   assign pc_plus_offset_aligned[0] = pc_plus_offset[0] & !i_cnt0;
   assign pc_plus_offset_cy_r_w[0] = pc_plus_offset_cy_r;
   assign pc_plus_4_cy_r_w[0] = pc_plus_4_cy_r;

   // Initialize o_ibus_adr to RESET_PC when using the "NONE" reset strategy.
   initial if (RESET_STRATEGY == "NONE") o_ibus_adr = RESET_PC;

   // On each clock cycle, update the carry registers and, if enabled (or reset),
   // update the instruction bus address.
   always @(posedge clk) begin
      pc_plus_4_cy_r <= i_pc_en & pc_plus_4_cy;
      pc_plus_offset_cy_r <= i_pc_en & pc_plus_offset_cy;
      if (RESET_STRATEGY == "NONE") begin
         if (i_pc_en)
            o_ibus_adr <= { new_pc, o_ibus_adr[31:W] };
      end else begin
         if (i_pc_en | i_rst)
            o_ibus_adr <= i_rst ? RESET_PC : { new_pc, o_ibus_adr[31:W] };
      end
   end

`ifdef BUNDLE_CTRL_OUTPUTS
   // Bundle the two data outputs (o_rd and o_bad_pc) into one bus.
   // The bus width is 2*(B+1) bits, with the upper half corresponding to o_rd and
   // the lower half corresponding to o_bad_pc.
   wire [2*(B+1)-1:0] ctrl_data_bus;
   assign ctrl_data_bus = { o_rd, o_bad_pc };
`endif

`ifdef BUNDLE_CTRL_OUTPUTS
   // If bundling is enabled, you may replace the separate ports for o_rd and o_bad_pc with
   // the bundled output. (Downstream modules should then extract the individual signals.)
   // For example, o_rd can be defined as ctrl_data_bus[2*(B+1)-1:B+1] and o_bad_pc as ctrl_data_bus[B:0].
`endif

endmodule
`default_nettype wire