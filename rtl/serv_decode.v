`default_nettype none
//----------------------------------------------------------------------
// Uncomment the following define to bundle decode outputs into a single bus.
// For production builds (to save IO), define this; for debugging, leave it undefined.
//----------------------------------------------------------------------
//`define BUNDLE_DECODE_IO

module serv_decode
  #(parameter [0:0] PRE_REGISTER = 1,
    parameter [0:0] MDU = 0)
  (
   input  wire        clk,
   // Input: Instruction word (bits 31:2) and enable signal.
   input  wire [31:2] i_wb_rdt,
   input  wire        i_wb_en,
   
`ifdef BUNDLE_DECODE_IO
   // When bundling is enabled, output a single bus.
   // The bus is 58 bits wide (see mapping below).
   // Mapping (from MSB to LSB):
   // [57] o_rd_mem_en
   // [56] o_rd_csr_en
   // [55] o_rd_alu_en
   // [54:51] o_immdec_ctrl (4 bits)
   // [50:47] o_immdec_en   (4 bits)
   // [46] o_op_b_source
   // [45] o_csr_imm_en
   // [44:43] o_csr_addr    (2 bits)
   // [42] o_csr_mstatus_en
   // [41] o_csr_mie_en
   // [40] o_csr_mcause_en
   // [39:38] o_csr_source  (2 bits)
   // [37] o_csr_d_sel
   // [36] o_mtval_pc
   // [35:33] o_alu_rd_sel  (3 bits)
   // [32] o_alu_cmp_sig
   // [31] o_alu_cmp_eq
   // [30:29] o_alu_bool_op (2 bits)
   // [28] o_alu_sub
   // [27] o_ctrl_mret
   // [26] o_ctrl_pc_rel
   // [25] o_ctrl_utype
   // [24] o_ctrl_jal_or_jalr
   // [23] o_bufreg_sh_signed
   // [22] o_bufreg_clr_lsb
   // [21] o_bufreg_imm_en
   // [20] o_bufreg_rs1_en
   // [19:17] o_ext_funct3   (3 bits)
   // [16] o_mdu_op
   // [15] o_dbus_en
   // [14] o_two_stage_op
   // [13] o_rd_op
   // [12] o_slt_or_branch
   // [11] o_shift_op
   // [10] o_branch_op
   // [9]  o_ebreak
   // [8]  o_e_op
   // [7]  o_cond_branch
   // [6]  o_bne_or_bge
   // [5]  o_sh_right
   // [4:3] o_immdec_ctrl (if needed extra bits; here we already used 4 bits above)
   // [2:0] reserved (or you can pack further if desired)
   // For this example, we assume a total width of 58 bits.
   output wire [57:0] o_decode_bus,
`else
   // When bundling is not enabled, use individual outputs.
   // To state:
   output reg       o_sh_right,
   output reg       o_bne_or_bge,
   output reg       o_cond_branch,
   output reg       o_e_op,
   output reg       o_ebreak,
   output reg       o_branch_op,
   output reg       o_shift_op,
   output reg       o_slt_or_branch,
   output reg       o_rd_op,
   output reg       o_two_stage_op,
   output reg       o_dbus_en,
   // MDU
   output reg       o_mdu_op,
   // Extension
   output reg [2:0] o_ext_funct3,
   // To bufreg
   output reg       o_bufreg_rs1_en,
   output reg       o_bufreg_imm_en,
   output reg       o_bufreg_clr_lsb,
   output reg       o_bufreg_sh_signed,
   // To ctrl
   output reg       o_ctrl_jal_or_jalr,
   output reg       o_ctrl_utype,
   output reg       o_ctrl_pc_rel,
   output reg       o_ctrl_mret,
   // To alu
   output reg       o_alu_sub,
   output reg [1:0] o_alu_bool_op,
   output reg       o_alu_cmp_eq,
   output reg       o_alu_cmp_sig,
   output reg [2:0] o_alu_rd_sel,
   // To mem IF
   output reg       o_mem_signed,
   output reg       o_mem_word,
   output reg       o_mem_half,
   output reg       o_mem_cmd,
   // To CSR
   output reg       o_csr_en,
   output reg [1:0] o_csr_addr,
   output reg       o_csr_mstatus_en,
   output reg       o_csr_mie_en,
   output reg       o_csr_mcause_en,
   output reg [1:0] o_csr_source,
   output reg       o_csr_d_sel,
   output reg       o_csr_imm_en,
   output reg       o_mtval_pc,
   // To top (immediate decoder control signals)
   output reg [3:0] o_immdec_ctrl,
   output reg [3:0] o_immdec_en,
   output reg       o_op_b_source,
   // To RF IF
   output reg       o_rd_mem_en,
   output reg       o_rd_csr_en,
   output reg       o_rd_alu_en
`endif
   );

   //-------------------------------------------------------------------------
   // Internal signal declarations for instruction fields
   //-------------------------------------------------------------------------
   reg [4:0] opcode;
   reg [2:0] funct3;
   reg        op20;
   reg        op21;
   reg        op22;
   reg        op26;

   reg        imm25;
   reg        imm30;

   //-------------------------------------------------------------------------
   // Combinational Decoding: Generate control signals based on instruction fields.
   // These "co_" wires are intermediate combinational results.
   //-------------------------------------------------------------------------
   wire co_mdu_op     = MDU & (opcode == 5'b01100) & imm25;

   wire co_two_stage_op =
         ~opcode[2] | (funct3[0] & ~funct3[1] & ~opcode[0] & ~opcode[4]) |
         (funct3[1] & ~funct3[2] & ~opcode[0] & ~opcode[4]) | co_mdu_op;
   wire co_shift_op = (opcode[2] & ~funct3[1]) & !co_mdu_op;
   wire co_slt_or_branch = (opcode[4] | (funct3[1] & opcode[2]) | (imm30 & opcode[2] & opcode[3] & ~funct3[2])) & !co_mdu_op;
   wire co_branch_op = opcode[4];
   wire co_dbus_en    = ~opcode[2] & ~opcode[4];
   wire co_mtval_pc   = opcode[4];
   wire co_mem_word   = funct3[1];
   wire co_rd_alu_en  = !opcode[0] & opcode[2] & !opcode[4] & !co_mdu_op;
   wire co_rd_mem_en  = (!opcode[2] & !opcode[0]) | co_mdu_op;
   wire [2:0] co_ext_funct3 = funct3;

   // Buffer control signals: which operand is buffered.
   wire co_bufreg_rs1_en = !opcode[4] | (!opcode[1] & opcode[0]);
   wire co_bufreg_imm_en = !opcode[2];

   // Clear LSB for branch and JAL ops.
   wire co_bufreg_clr_lsb = opcode[4] & ((opcode[1:0] == 2'b00) | (opcode[1:0] == 2'b11));

   // Conditional branch flag: true for branch instructions.
   wire co_cond_branch = !opcode[0];

   // Control signals for jump/branch types.
   wire co_ctrl_utype       = !opcode[4] & opcode[2] & opcode[0];
   wire co_ctrl_jal_or_jalr = opcode[4] & opcode[0];

   // PC-relative flag.
   wire co_ctrl_pc_rel = (opcode[2:0] == 3'b000)  |
                         (opcode[1:0] == 2'b11)  |
                         (opcode[4] & opcode[2] & op20) |
                         (opcode[4:3] == 2'b00);
   // Destination register write enable.
   wire co_rd_op = (opcode[2] |
                     (!opcode[2] & opcode[4] & opcode[0]) |
                     (!opcode[2] & !opcode[3] & !opcode[0]));

   // From funct3 field.
   wire co_sh_right   = funct3[2];
   wire co_bne_or_bge = funct3[0];

   // CSR operations.
   wire csr_op = opcode[4] & opcode[2] & (|funct3);

   // op20 used for ebreak.
   wire co_ebreak = op20;

   // Control for mret.
   wire co_ctrl_mret = opcode[4] & opcode[2] & op21 & !(|funct3);
   // Exception operation.
   wire co_e_op = opcode[4] & opcode[2] & !op21 & !(|funct3);

   // Buffer shift-signed flag.
   wire co_bufreg_sh_signed = imm30;

   // ALU subtraction.
   wire co_alu_sub = funct3[1] | funct3[0] | (opcode[3] & imm30) | opcode[4];

   // CSR-related signals.
   wire csr_valid = op20 | (op26 & !op21);
   wire co_rd_csr_en = csr_op;
   wire co_csr_en         = csr_op & csr_valid;
   wire co_csr_mstatus_en = csr_op & !op26 & !op22 & !op20;
   wire co_csr_mie_en     = csr_op & !op26 &  op22 & !op20;
   wire co_csr_mcause_en  = csr_op & op21 & !op20;
   wire [1:0] co_csr_source = funct3[1:0];
   wire co_csr_d_sel = funct3[2];
   wire co_csr_imm_en = opcode[4] & opcode[2] & funct3[2];
   wire [1:0] co_csr_addr = {op26 & op20, !op26 | op21};

   // ALU comparisons.
   wire co_alu_cmp_eq = (funct3[2:1] == 2'b00);
   wire co_alu_cmp_sig = ~((funct3[0] & funct3[1]) | (funct3[1] & funct3[2]));

   // Memory command signals.
   wire co_mem_cmd  = opcode[3];
   wire co_mem_signed = ~funct3[2];
   wire co_mem_half   = funct3[0];

   wire [1:0] co_alu_bool_op = funct3[1:0];

   // Immediate decoder control signals.
   wire [3:0] co_immdec_ctrl;
   assign co_immdec_ctrl[0] = (opcode[3:0] == 4'b1000);
   assign co_immdec_ctrl[1] = ((opcode[1:0] == 2'b00) | (opcode[2:1] == 2'b00));
   assign co_immdec_ctrl[2] = opcode[4] & ~opcode[0];
   assign co_immdec_ctrl[3] = opcode[4];

   wire [3:0] co_immdec_en;
   assign co_immdec_en[3] = opcode[4] | opcode[3] | opcode[2] | ~opcode[0];
   assign co_immdec_en[2] = (opcode[4] & opcode[2]) | ~opcode[3] | opcode[0];
   assign co_immdec_en[1] = ((opcode[2:1] == 2'b01) | (opcode[2] & opcode[0]) | co_csr_imm_en);
   assign co_immdec_en[0] = ~co_rd_op;

   wire [2:0] co_alu_rd_sel;
   assign co_alu_rd_sel[0] = (funct3 == 3'b000);
   assign co_alu_rd_sel[1] = (funct3[2:1] == 2'b01);
   assign co_alu_rd_sel[2] = funct3[2];

   // Select source for operand B.
   wire co_op_b_source = opcode[3];

   //-------------------------------------------------------------------------
   // Generate block: Pre-register vs. Post-register mode.
   //-------------------------------------------------------------------------
   generate
      if (PRE_REGISTER) begin : gen_pre_register

         always @(posedge clk) begin
            if (i_wb_en) begin
               funct3 <= i_wb_rdt[14:12];
               imm30  <= i_wb_rdt[30];
               imm25  <= i_wb_rdt[25];
               opcode <= i_wb_rdt[6:2];
               op20   <= i_wb_rdt[20];
               op21   <= i_wb_rdt[21];
               op22   <= i_wb_rdt[22];
               op26   <= i_wb_rdt[26];
            end
         end

         always @(*) begin
            o_sh_right         = co_sh_right;
            o_bne_or_bge       = co_bne_or_bge;
            o_cond_branch      = co_cond_branch;
            o_dbus_en          = co_dbus_en;
            o_mtval_pc         = co_mtval_pc;
            o_two_stage_op     = co_two_stage_op;
            o_e_op             = co_e_op;
            o_ebreak           = co_ebreak;
            o_branch_op        = co_branch_op;
            o_shift_op         = co_shift_op;
            o_slt_or_branch    = co_slt_or_branch;
            o_rd_op            = co_rd_op;
            o_mdu_op           = co_mdu_op;
            o_ext_funct3       = co_ext_funct3;
            o_bufreg_rs1_en    = co_bufreg_rs1_en;
            o_bufreg_imm_en    = co_bufreg_imm_en;
            o_bufreg_clr_lsb   = co_bufreg_clr_lsb;
            o_bufreg_sh_signed = co_bufreg_sh_signed;
            o_ctrl_jal_or_jalr = co_ctrl_jal_or_jalr;
            o_ctrl_utype       = co_ctrl_utype;
            o_ctrl_pc_rel      = co_ctrl_pc_rel;
            o_ctrl_mret        = co_ctrl_mret;
            o_alu_sub          = co_alu_sub;
            o_alu_bool_op      = co_alu_bool_op;
            o_alu_cmp_eq       = co_alu_cmp_eq;
            o_alu_cmp_sig      = co_alu_cmp_sig;
            o_alu_rd_sel       = co_alu_rd_sel;
            o_mem_signed       = co_mem_signed;
            o_mem_word         = co_mem_word;
            o_mem_half         = co_mem_half;
            o_mem_cmd          = co_mem_cmd;
            o_csr_en           = co_csr_en;
            o_csr_addr         = co_csr_addr;
            o_csr_mstatus_en   = co_csr_mstatus_en;
            o_csr_mie_en       = co_csr_mie_en;
            o_csr_mcause_en    = co_csr_mcause_en;
            o_csr_source       = co_csr_source;
            o_csr_d_sel        = co_csr_d_sel;
            o_csr_imm_en       = co_csr_imm_en;
            o_immdec_ctrl      = co_immdec_ctrl;
            o_immdec_en        = co_immdec_en;
            o_op_b_source      = co_op_b_source;
            o_rd_csr_en        = co_rd_csr_en;
            o_rd_alu_en        = co_rd_alu_en;
            o_rd_mem_en        = co_rd_mem_en;
         end

      end else begin : gen_post_register

         always @(*) begin
            funct3  = i_wb_rdt[14:12];
            imm30   = i_wb_rdt[30];
            imm25   = i_wb_rdt[25];
            opcode  = i_wb_rdt[6:2];
            op20    = i_wb_rdt[20];
            op21    = i_wb_rdt[21];
            op22    = i_wb_rdt[22];
            op26    = i_wb_rdt[26];
         end

         always @(posedge clk) begin
            if (i_wb_en) begin
               o_sh_right         <= co_sh_right;
               o_bne_or_bge       <= co_bne_or_bge;
               o_cond_branch      <= co_cond_branch;
               o_e_op             <= co_e_op;
               o_ebreak           <= co_ebreak;
               o_two_stage_op     <= co_two_stage_op;
               o_dbus_en          <= co_dbus_en;
               o_mtval_pc         <= co_mtval_pc;
               o_branch_op        <= co_branch_op;
               o_shift_op         <= co_shift_op;
               o_slt_or_branch    <= co_slt_or_branch;
               o_rd_op            <= co_rd_op;
               o_mdu_op           <= co_mdu_op;
               o_ext_funct3       <= co_ext_funct3;
               o_bufreg_rs1_en    <= co_bufreg_rs1_en;
               o_bufreg_imm_en    <= co_bufreg_imm_en;
               o_bufreg_clr_lsb   <= co_bufreg_clr_lsb;
               o_bufreg_sh_signed <= co_bufreg_sh_signed;
               o_ctrl_jal_or_jalr <= co_ctrl_jal_or_jalr;
               o_ctrl_utype       <= co_ctrl_utype;
               o_ctrl_pc_rel      <= co_ctrl_pc_rel;
               o_ctrl_mret        <= co_ctrl_mret;
               o_alu_sub          <= co_alu_sub;
               o_alu_bool_op      <= co_alu_bool_op;
               o_alu_cmp_eq       <= co_alu_cmp_eq;
               o_alu_cmp_sig      <= co_alu_cmp_sig;
               o_alu_rd_sel       <= co_alu_rd_sel;
               o_mem_signed       <= co_mem_signed;
               o_mem_word         <= co_mem_word;
               o_mem_half         <= co_mem_half;
               o_mem_cmd          <= co_mem_cmd;
               o_csr_en           <= co_csr_en;
               o_csr_addr         <= co_csr_addr;
               o_csr_mstatus_en   <= co_csr_mstatus_en;
               o_csr_mie_en       <= co_csr_mie_en;
               o_csr_mcause_en    <= co_csr_mcause_en;
               o_csr_source       <= co_csr_source;
               o_csr_d_sel        <= co_csr_d_sel;
               o_csr_imm_en       <= co_csr_imm_en;
               o_immdec_ctrl      <= co_immdec_ctrl;
               o_immdec_en        <= co_immdec_en;
               o_op_b_source      <= co_op_b_source;
               o_rd_csr_en        <= co_rd_csr_en;
               o_rd_alu_en        <= co_rd_alu_en;
               o_rd_mem_en        <= co_rd_mem_en;
            end
         end

      end
   endgenerate

`ifdef BUNDLE_DECODE_IO
   // When bundling is enabled, pack the individual outputs into one bus.
   // The following mapping is used (from MSB [57] to LSB [0]):
   // [57] o_rd_mem_en
   // [56] o_rd_csr_en
   // [55] o_rd_alu_en
   // [54:51] o_immdec_ctrl (4 bits)
   // [50:47] o_immdec_en   (4 bits)
   // [46] o_op_b_source
   // [45] o_csr_imm_en
   // [44:43] o_csr_addr (2 bits)
   // [42] o_csr_mstatus_en
   // [41] o_csr_mie_en
   // [40] o_csr_mcause_en
   // [39:38] o_csr_source (2 bits)
   // [37] o_csr_d_sel
   // [36] o_mtval_pc
   // [35:33] o_alu_rd_sel (3 bits)
   // [32] o_alu_cmp_sig
   // [31] o_alu_cmp_eq
   // [30:29] o_alu_bool_op (2 bits)
   // [28] o_alu_sub
   // [27] o_ctrl_mret
   // [26] o_ctrl_pc_rel
   // [25] o_ctrl_utype
   // [24] o_ctrl_jal_or_jalr
   // [23] o_bufreg_sh_signed
   // [22] o_bufreg_clr_lsb
   // [21] o_bufreg_imm_en
   // [20] o_bufreg_rs1_en
   // [19:17] o_ext_funct3 (3 bits)
   // [16] o_mdu_op
   // [15] o_dbus_en
   // [14] o_two_stage_op
   // [13] o_rd_op
   // [12] o_slt_or_branch
   // [11] o_shift_op
   // [10] o_branch_op
   // [9]  o_ebreak
   // [8]  o_e_op
   // [7]  o_cond_branch
   // [6]  o_bne_or_bge
   // [5]  o_sh_right
   // [4:0] reserved (or you can pack further if desired)
   // (Total used bits: 58, with 5 bits reserved to round to 63 if needed; here we assume exactly 58 bits.)
   // For this example, we pack 58 bits.
   wire [57:0] decode_bus;
   assign decode_bus = { o_rd_mem_en,
                         o_rd_csr_en,
                         o_rd_alu_en,
                         o_immdec_ctrl,      // 4 bits
                         o_immdec_en,        // 4 bits
                         o_op_b_source,
                         o_csr_imm_en,
                         o_csr_addr,         // 2 bits
                         o_csr_mstatus_en,
                         o_csr_mie_en,
                         o_csr_mcause_en,
                         o_csr_source,       // 2 bits
                         o_csr_d_sel,
                         o_mtval_pc,
                         o_alu_rd_sel,       // 3 bits
                         o_alu_cmp_sig,
                         o_alu_cmp_eq,
                         o_alu_bool_op,      // 2 bits
                         o_alu_sub,
                         o_ctrl_mret,
                         o_ctrl_pc_rel,
                         o_ctrl_utype,
                         o_ctrl_jal_or_jalr,
                         o_bufreg_sh_signed,
                         o_bufreg_clr_lsb,
                         o_bufreg_imm_en,
                         o_bufreg_rs1_en,
                         o_ext_funct3,       // 3 bits
                         o_mdu_op,
                         o_dbus_en,
                         o_two_stage_op,
                         o_rd_op,
                         o_slt_or_branch,
                         o_shift_op,
                         o_branch_op,
                         o_ebreak,
                         o_e_op,
                         o_cond_branch,
                         o_bne_or_bge,
                         o_sh_right
                       };
   // Connect the bundled bus to the output.
   assign o_decode_bus = decode_bus;
`endif

endmodule
`default_nettype wire