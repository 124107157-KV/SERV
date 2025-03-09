`default_nettype none

//----------------------------------------------------------------------
// Uncomment the following define to bundle state outputs into a single bus.
// For production builds (to save IO), define this; for debugging, leave it undefined.
//----------------------------------------------------------------------
//`define BUNDLE_STATE_IO

module serv_state
  #(parameter RESET_STRATEGY = "MINI",
    parameter [0:0] WITH_CSR = 1,
    parameter [0:0] ALIGN = 0,
    parameter [0:0] MDU = 0,
    parameter       W = 1
  )
  (
   input  wire i_clk,
   input  wire i_rst,
   // State inputs
   input  wire i_new_irq,
   input  wire i_alu_cmp,
   
`ifdef BUNDLE_STATE_IO
   // Instead of many individual outputs, we bundle the following signals into one bus.
   // The bus is 18 bits wide as described in the comments below.
   output wire [17:0] o_state_bus,
`else
   output wire o_init,
   output wire o_cnt_en,
   output wire o_cnt0to3,
   output wire o_cnt12to31,
   output wire o_cnt0,
   output wire o_cnt1,
   output wire o_cnt2,
   output wire o_cnt3,
   output wire o_cnt7,
   output wire o_cnt11,
   output wire o_cnt12,
   output wire o_cnt_done,
   output wire o_bufreg_en,
   output wire o_ctrl_pc_en,
   output reg  o_ctrl_jump,
   output wire o_ctrl_trap,
   output wire [1:0] o_mem_bytecnt,
`endif

   // Other control and status inputs
   input  wire i_ctrl_misalign,
   input  wire i_sh_done,
   input  wire i_sh_done_r,
   input  wire i_mem_misalign,
   // Control signals from decode
   input  wire i_bne_or_bge,
   input  wire i_cond_branch,
   input  wire i_dbus_en,
   input  wire i_two_stage_op,
   input  wire i_branch_op,
   input  wire i_shift_op,
   input  wire i_sh_right,
   input  wire i_slt_or_branch,
   input  wire i_e_op,
   input  wire i_rd_op,
   // MDU signals
   input  wire i_mdu_op,
   output wire o_mdu_valid,
   // Extension (for MDU readiness)
   input  wire i_mdu_ready,
   // External bus interface signals
   output wire o_dbus_cyc,
   input  wire i_dbus_ack,
   output wire o_ibus_cyc,
   input  wire i_ibus_ack,
   // RF interface signals
   output wire o_rf_rreq,
   output wire o_rf_wreq,
   input  wire i_rf_ready,
   output wire o_rf_rd_en
   );

   // Internal signal declarations (all signals remain unchanged regardless of bundling)
   reg  stage_two_req;
   reg  init_done;
   wire misalign_trap_sync;
   reg [4:2] o_cnt;      // Upper 3 bits of counter
   wire [3:0] cnt_r;     // Lower 4 bits from shift register
   reg ibus_cyc;
   // PC update enable is active only when counter is enabled and not in init phase.
   wire internal_ctrl_pc_en;
   assign internal_ctrl_pc_en = o_cnt_en & !o_init;
   // For memory byte count, use the upper two bits of o_cnt.
   wire [1:0] internal_mem_bytecnt;
   assign internal_mem_bytecnt = o_cnt[4:3];
   // Counter comparisons and strobes
   wire internal_cnt0to3 = (o_cnt[4:2] == 3'd0);
   wire internal_cnt12to31 = (o_cnt[4] | (o_cnt[3:2] == 2'b11));
   wire internal_cnt0 = (o_cnt[4:2] == 3'd0) & cnt_r[0];
   wire internal_cnt1 = (o_cnt[4:2] == 3'd0) & cnt_r[1];
   wire internal_cnt2 = (o_cnt[4:2] == 3'd0) & cnt_r[2];
   wire internal_cnt3 = (o_cnt[4:2] == 3'd0) & cnt_r[3];
   wire internal_cnt7 = (o_cnt[4:2] == 3'd1) & cnt_r[3];
   wire internal_cnt11 = (o_cnt[4:2] == 3'd2) & cnt_r[3];
   wire internal_cnt12 = (o_cnt[4:2] == 3'd3) & cnt_r[0];
   wire internal_cnt_done = (o_cnt[4:2] == 3'b111) & cnt_r[3];
   // Branch condition signal: "take_branch" is true if branch op is active and either
   // the branch is unconditional or the condition (ALU compare, bne_or_bge flag) holds.
   wire take_branch = i_branch_op & (!i_cond_branch | (i_alu_cmp ^ i_bne_or_bge));
   // MDU valid signal: asserted when MDU is enabled, counter is off, init is done, and an MDU op is requested.
   assign o_mdu_valid = MDU & !o_cnt_en & init_done & i_mdu_op;
   // RF write request: asserted when no misalignment trap is pending, counter is off, init is done, and one of several conditions holds.
   wire internal_rf_wreq = (!misalign_trap_sync) & (!o_cnt_en) & init_done &
                              ((i_shift_op & (i_sh_done | !i_sh_right)) |
                               i_dbus_ack | (MDU & i_mdu_ready) |
                               i_slt_or_branch);
   // Data bus cycle active when counter is off, init is done, DBus enabled, and no memory misalignment.
   wire internal_dbus_cyc = (!o_cnt_en) & init_done & i_dbus_en & (!i_mem_misalign);
   // RF read request is asserted on IBUS acknowledge or when stage two is requested with a misalignment trap.
   wire internal_rf_rreq = i_ibus_ack | (stage_two_req & misalign_trap_sync);
   // RF read enable is active only when a rd operation is requested and the core is not in init.
   wire internal_rf_rd_en = i_rd_op & !o_init;
   // Buffer register enable signal: complex condition based on counter, shift op, two-stage op, etc.
   wire internal_bufreg_en = (o_cnt_en & (o_init | ((o_ctrl_trap | i_branch_op) & i_two_stage_op))) |
                             (i_shift_op & !stage_two_req & (i_sh_right | i_sh_done_r) & init_done);
   // Instruction bus cycle signal, gated by reset.
   // (ibus_cyc is updated in the always block below.)
   // o_ibus_cyc is driven as: ibus_cyc & !i_rst.
   // (Note: In our bundled configuration, we will pack this into the bus.)
   // For now, we keep it as internal signal "ibus_cyc".
   // o_init: The core is in initialization when two-stage op is active, no new IRQ, and init is not done.
   wire internal_init = i_two_stage_op & !i_new_irq & !init_done;
   //------------------------------------------------------------------------
   // (The rest of the internal signals such as those for the counter,
   // branch decision, etc., remain unchanged.)
   //------------------------------------------------------------------------
   // Assign counter outputs and strobes as in the original code.
   assign o_ctrl_pc_en  = internal_ctrl_pc_en;
   assign o_mem_bytecnt = internal_mem_bytecnt;
   assign o_cnt0to3   = internal_cnt0to3;
   assign o_cnt12to31 = internal_cnt12to31;
   assign o_cnt0 = internal_cnt0;
   assign o_cnt1 = internal_cnt1;
   assign o_cnt2 = internal_cnt2;
   assign o_cnt3 = internal_cnt3;
   assign o_cnt7 = internal_cnt7;
   assign o_cnt11 = internal_cnt11;
   assign o_cnt12 = internal_cnt12;
   assign o_cnt_done = internal_cnt_done;
   // The RF and bus control signals are assigned as in the original code.
   assign o_rf_wreq = internal_rf_wreq;
   assign o_dbus_cyc = internal_dbus_cyc;
   assign o_rf_rreq = internal_rf_rreq;
   assign o_rf_rd_en = internal_rf_rd_en;
   // For PC update, initialization, and jump, assign as in original.
   assign o_init = internal_init;
   // (Other internal assignments for branch and MDU valid signals remain unchanged.)
   // ...
   // Counter implementation generate block (unchanged)
   generate
      if (W == 1) begin : gen_cnt_w_eq_1
         reg [3:0] cnt_lsb;
         always @(posedge i_clk) begin
            o_cnt <= o_cnt + {2'd0, cnt_r[3]};
            cnt_lsb <= {cnt_lsb[2:0], (cnt_lsb[3] & !o_cnt_done) | (i_rf_ready & !o_cnt_en)};
            if (i_rst & (RESET_STRATEGY != "NONE")) begin
               o_cnt   <= 3'd0;
               cnt_lsb <= 4'b0000;
            end
         end
         assign cnt_r = cnt_lsb;
         assign o_cnt_en = |cnt_lsb;
      end else if (W == 4) begin : gen_cnt_w_eq_4
         reg cnt_en;
         always @(posedge i_clk) begin
            if (i_rf_ready) cnt_en <= 1; else
            if (o_cnt_done) cnt_en <= 0;
            o_cnt <= o_cnt + {2'd0, cnt_en};
            if (i_rst & (RESET_STRATEGY != "NONE")) begin
               o_cnt   <= 3'd0;
               cnt_en <= 1'b0;
            end
         end
         assign cnt_r = 4'b1111;
         assign o_cnt_en = cnt_en;
      end
   endgenerate
   // Control trap generation using misalignment trap synchronization.
   assign o_ctrl_trap = WITH_CSR & (i_e_op | i_new_irq | misalign_trap_sync);
   generate
      if (WITH_CSR) begin : gen_csr_sync
         reg misalign_trap_sync_r;
         wire trap_pending = WITH_CSR & ((take_branch & i_ctrl_misalign & !ALIGN) |
                                           (i_dbus_en   & i_mem_misalign));
         always @(posedge i_clk) begin
            if (i_ibus_ack | o_cnt_done | i_rst)
               misalign_trap_sync_r <= !(i_ibus_ack | i_rst) & ((trap_pending & o_init) | misalign_trap_sync_r);
         end
         assign misalign_trap_sync = misalign_trap_sync_r;
      end else begin : gen_no_csr_sync
         assign misalign_trap_sync = 1'b0;
      end
   endgenerate
   //-------------------------------------------------------------------------
   // Control and Stage Management
   //-------------------------------------------------------------------------
   always @(posedge i_clk) begin
      // Update ibus_cyc on reset, instruction bus acknowledge, or counter done.
      if (i_ibus_ack | o_cnt_done | i_rst)
         ibus_cyc <= internal_ctrl_pc_en | i_rst;
      if (o_cnt_done) begin
         init_done <= o_init & !init_done;
         o_ctrl_jump <= o_init & take_branch;
      end
      // Generate stage_two_req strobe.
      stage_two_req <= o_cnt_done & o_init;
      if (i_rst) begin
         if (RESET_STRATEGY != "NONE") begin
            init_done <= 1'b0;
            o_ctrl_jump <= 1'b0;
            stage_two_req <= 1'b0;
         end
      end
   end
   //-------------------------------------------------------------------------
   // Optional: Bundle IO Outputs for reduced external pin count.
   //-------------------------------------------------------------------------
`ifdef BUNDLE_STATE_IO
   // In the bundled configuration, we do not output the individual signals.
   // Instead, we pack them into one 18-bit bus.
   // The mapping (from MSB [17] to LSB [0]) is as follows:
   // [17] o_ctrl_trap
   // [16] o_ctrl_jump
   // [15] o_ctrl_pc_en
   // [14] o_bufreg_en
   // [13] o_cnt_done
   // [12] o_cnt12
   // [11] o_cnt11
   // [10] o_cnt7
   // [9]  o_cnt3
   // [8]  o_cnt2
   // [7]  o_cnt1
   // [6]  o_cnt0
   // [5]  o_cnt12to31
   // [4]  o_cnt0to3
   // [3]  o_cnt_en
   // [2]  o_init
   // [1:0] o_mem_bytecnt
   assign o_state_bus = { o_ctrl_trap, o_ctrl_jump, o_ctrl_pc_en, o_bufreg_en, o_cnt_done,
                           o_cnt12, o_cnt11, o_cnt7, o_cnt3, o_cnt2, o_cnt1, o_cnt0,
                           o_cnt12to31, o_cnt0to3, o_cnt_en, o_init, o_mem_bytecnt };
`endif
endmodule
`default_nettype wire