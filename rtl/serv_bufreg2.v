`default_nettype none
//----------------------------------------------------------------------
// Uncomment the following define to bundle serv_bufreg2 outputs into fewer ports.
// For production builds (to save IO), define this; for debugging, leave it undefined.
//----------------------------------------------------------------------
//`define BUNDLE_BUFREG2_IO

module serv_bufreg2
  (
   input wire        i_clk,
   // State signals
   input wire        i_en,
   input wire        i_init,
   input wire        i_cnt_done,
   input wire [1:0]  i_lsb,
   input wire        i_byte_valid,
   // Control signals
   input wire        i_op_b_sel,
   input wire        i_shift_op,
   // Data signals
   input wire        i_rs2,
   input wire        i_imm,
   output wire       o_op_b,
   output wire       o_q,
`ifdef BUNDLE_BUFREG2_IO
   // Instead of separate outputs, bundle the small control outputs into one bus.
   // We'll bundle the following 4 one-bit signals:
   // [3] = o_sh_done, [2] = o_sh_done_r, [1] = o_op_b, [0] = o_q.
   output wire [3:0] o_small_bus,
`else
   output wire       o_sh_done,
   output wire       o_sh_done_r,
`endif
   // External data output
`ifdef BUNDLE_BUFREG2_IO
   // In the bundled configuration, the 32-bit data output is merged with the small bus
   // into a single 36-bit bus: [35:4] = o_dat, [3:0] = o_small_bus.
   output wire [35:0] o_bufreg2_bus,
`else
   output wire [31:0] o_dat,
`endif
   // Additional external inputs
   input wire        i_load,
   input wire [31:0] i_dat
   );

   //-------------------------------------------------------------------------
   // Internal Logic (Unchanged)
   //-------------------------------------------------------------------------
   // The module implements a shift register (dat) that holds 32 bits.
   // It also computes the result of adding (masked) i_rs2 and i_imm (selectable by i_op_b_sel)
   // and then shifts the result into the data register.
   reg [31:0] dat;
   
   // o_op_b is selected based on i_op_b_sel.
   assign o_op_b = i_op_b_sel ? i_rs2 : i_imm;
   
   // Generate a "data enable" signal: update occurs when a shift operation is active
   // or when both i_en and i_byte_valid are asserted.
   wire dat_en = i_shift_op | (i_en & i_byte_valid);
   
   /* 
    The 'dat' register serves three purposes:
      - For store operations, new data is shifted into 'dat' during initialization.
      - For load operations, external data (i_dat) is latched into 'dat'.
      - For shift operations, after initialization, the lower 6 bits of 'dat' act as a downcounter,
        triggering o_sh_done and o_sh_done_r when they wrap.
   */
   wire [5:0] dat_shamt = (i_shift_op & !i_init) ?
                           // Down counter mode: subtract 1 from the lower 6 bits.
                           dat[5:0] - 1 :
                           // Shift-register mode with optional clearing of bit 5.
                           {dat[6] & !(i_shift_op & i_cnt_done), dat[5:1]};
   
   // Generate the shift-done signals.
   assign o_sh_done = dat_shamt[5];
   assign o_sh_done_r = dat[5];
   
   // Generate the serial output bit o_q using i_lsb to select one bit from dat.
   // For different values of i_lsb, select bits [24], [16], [8], or [0] respectively.
   assign o_q =
         ((i_lsb == 2'd3) & dat[24]) |
         ((i_lsb == 2'd2) & dat[16]) |
         ((i_lsb == 2'd1) & dat[8])  |
         ((i_lsb == 2'd0) & dat[0]);
   
`ifndef BUNDLE_BUFREG2_IO
   // In the unbundled version, o_dat is output separately.
   assign o_dat = dat;
`endif

   //-------------------------------------------------------------------------
   // Data Register Update
   //-------------------------------------------------------------------------
   // On each rising clock edge, if dat_en or i_load is asserted, update 'dat'.
   always @(posedge i_clk) begin
      if (dat_en | i_load)
         dat <= i_load ? i_dat : {o_op_b, dat[31:7], dat_shamt};
   end

`ifdef BUNDLE_BUFREG2_IO
   //-------------------------------------------------------------------------
   // Bundled Output Assignment
   //-------------------------------------------------------------------------
   // Here we bundle the small control outputs and the 32-bit data output into a single 36-bit bus.
   // Mapping (from MSB to LSB):
   // [35:4] = dat (the 32-bit buffered value)
   // [3]    = o_sh_done
   // [2]    = o_sh_done_r
   // [1]    = o_op_b
   // [0]    = o_q
   wire [3:0] small_bus;
   assign small_bus = { o_sh_done, o_sh_done_r, o_op_b, o_q };
   
   // Bundle the outputs: the 36-bit bus is the concatenation of the 32-bit 'dat' and the 4-bit small bus.
   assign o_bufreg2_bus = { dat, small_bus };
`endif

endmodule
`default_nettype wire