`default_nettype none
//----------------------------------------------------------------------
// Uncomment the following define to bundle immdec outputs into a single bus.
// For production builds (to save IO), define this; for debugging, leave it undefined.
//----------------------------------------------------------------------
//`define BUNDLE_IMMDEC_IO

module serv_immdec
  #(parameter SHARED_RFADDR_IMM_REGS = 1)
  (
   input wire        i_clk,
   // State
   input wire        i_cnt_en,
   input wire        i_cnt_done,
   // Control
   input wire [3:0]  i_immdec_en,
   input wire        i_csr_imm_en,
   input wire [3:0]  i_ctrl,
   
`ifdef BUNDLE_IMMDEC_IO
   // When bundling is enabled, the three 5-bit register address outputs and
   // the two 1-bit immediate outputs are concatenated into one 17-bit bus.
   // Mapping (from MSB to LSB):
   // [16:12] = o_rd_addr (5 bits)
   // [11:7]  = o_rs1_addr (5 bits)
   // [6:2]   = o_rs2_addr (5 bits)
   // [1]     = o_csr_imm   (1 bit)
   // [0]     = o_imm       (1 bit)
   output wire [16:0] o_immdec_bus,
`else
   output wire [4:0] o_rd_addr,
   output wire [4:0] o_rs1_addr,
   output wire [4:0] o_rs2_addr,
   // Data
   output wire       o_csr_imm,
   output wire       o_imm,
`endif

   // External
   input wire        i_wb_en,
   input wire [31:7] i_wb_rdt
   );

   //-------------------------------------------------------------------------
   // Internal Registers for Immediate Extraction and (optional) Register Addresses
   //-------------------------------------------------------------------------
   reg         imm31;           // Bit 31 (sign bit) of the instruction.
   reg [8:0]   imm19_12_20;     // Bits 19:12 concatenated with bit 20.
   reg         imm7;            // Bit 7.
   reg [5:0]   imm30_25;        // Bits 30:25.
   reg [4:0]   imm24_20;        // Bits 24:20.
   reg [4:0]   imm11_7;         // Bits 11:7.

   // o_csr_imm is taken from bit 4 of imm19_12_20.
   assign o_csr_imm = imm19_12_20[4];

   // The effective sign bit is used unless CSR immediates are used.
   wire signbit = imm31 & ~i_csr_imm_en;

   //-------------------------------------------------------------------------
   // Generate Block: Shared versus Separate Immediate/Address Registers
   //-------------------------------------------------------------------------
   generate
      if (SHARED_RFADDR_IMM_REGS) begin : gen_shared_imm_regs
         // In shared mode, the register addresses are extracted from the immediate registers.
         assign o_rs1_addr = imm19_12_20[8:4];
         assign o_rs2_addr = imm24_20;
         assign o_rd_addr  = imm11_7;

         always @(posedge i_clk) begin
            if (i_wb_en) begin
               // Load the new instruction fields.
               /* CSR immediates are always zero-extended, hence clear the sign bit */
               imm31     <= i_wb_rdt[31];
            end
            if (i_wb_en | (i_cnt_en & i_immdec_en[1]))
              imm19_12_20 <= i_wb_en ? { i_wb_rdt[19:12], i_wb_rdt[20] }
                                      : { i_ctrl[3] ? signbit : imm24_20[0], imm19_12_20[8:1] };
            if (i_wb_en | i_cnt_en)
              imm7        <= i_wb_en ? i_wb_rdt[7] : signbit;
            if (i_wb_en | (i_cnt_en & i_immdec_en[3]))
              imm30_25    <= i_wb_en ? i_wb_rdt[30:25]
                                      : { i_ctrl[2] ? imm7 : i_ctrl[1] ? signbit : imm19_12_20[0], imm30_25[5:1] };
            if (i_wb_en | (i_cnt_en & i_immdec_en[2]))
              imm24_20    <= i_wb_en ? i_wb_rdt[24:20]
                                      : { imm30_25[0], imm24_20[4:1] };
            if (i_wb_en | (i_cnt_en & i_immdec_en[0]))
              imm11_7     <= i_wb_en ? i_wb_rdt[11:7]
                                      : { imm30_25[0], imm11_7[4:1] };
         end
      end else begin : gen_separate_imm_regs
         // In separate mode, additional registers hold the RF addresses.
         reg [4:0] rd_addr;
         reg [4:0] rs1_addr;
         reg [4:0] rs2_addr;

         assign o_rd_addr  = rd_addr;
         assign o_rs1_addr = rs1_addr;
         assign o_rs2_addr = rs2_addr;

         always @(posedge i_clk) begin
            if (i_wb_en) begin
               imm31       <= i_wb_rdt[31];
               imm19_12_20 <= { i_wb_rdt[19:12], i_wb_rdt[20] };
               imm7        <= i_wb_rdt[7];
               imm30_25    <= i_wb_rdt[30:25];
               imm24_20    <= i_wb_rdt[24:20];
               imm11_7     <= i_wb_rdt[11:7];

               rd_addr  <= i_wb_rdt[11:7];
               rs1_addr <= i_wb_rdt[19:15];
               rs2_addr <= i_wb_rdt[24:20];
            end
            if (i_cnt_en) begin
               imm19_12_20 <= { i_ctrl[3] ? signbit : imm24_20[0], imm19_12_20[8:1] };
               imm7        <= signbit;
               imm30_25    <= { i_ctrl[2] ? imm7 : i_ctrl[1] ? signbit : imm19_12_20[0], imm30_25[5:1] };
               imm24_20    <= { imm30_25[0], imm24_20[4:1] };
               imm11_7     <= { imm30_25[0], imm11_7[4:1] };
            end
         end
      end
   endgenerate

   //-------------------------------------------------------------------------
   // Final Immediate Output Selection
   //-------------------------------------------------------------------------
   // The final immediate (o_imm) is selected as follows:
   // - If i_cnt_done is asserted, output the signbit.
   // - Otherwise, if i_ctrl[0] is high, use the LSB of imm11_7;
   //   if low, use the LSB of imm24_20.
   assign o_imm = i_cnt_done ? signbit : i_ctrl[0] ? imm11_7[0] : imm24_20[0];

`ifdef BUNDLE_IMMDEC_IO
   // When bundling is enabled, combine the register address outputs and the immediate outputs
   // into a single bus. The mapping is as follows (from MSB to LSB):
   // [16:12] = o_rd_addr (5 bits)
   // [11:7]  = o_rs1_addr (5 bits)
   // [6:2]   = o_rs2_addr (5 bits)
   // [1]     = o_csr_imm  (1 bit)
   // [0]     = o_imm      (1 bit)
   // Note: In both shared and separate modes, o_csr_imm and o_imm are assigned above.
   wire [16:0] immdec_bus;
   assign immdec_bus = { o_rd_addr, o_rs1_addr, o_rs2_addr, o_csr_imm, o_imm };

   // Expose the bundled bus as the module output.
   // (If bundling is enabled, the external interface now uses one 17-bit bus instead of five separate outputs.)
   // In your top-level instantiation and constraints, use o_immdec_bus.
   // (The individual signals can be recovered internally by slicing the bus if needed.)
   // For example, o_rd_addr = immdec_bus[16:12], etc.
`endif
endmodule
`default_nettype wire