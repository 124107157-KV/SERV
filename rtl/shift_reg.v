module shift_reg
  (
   input wire 		 clk,
   input wire 		 i_en,
   input wire 		 i_d,
   output wire 		 o_q,
   output wire [LEN-2:0] o_par);

   parameter LEN = 0;
   parameter INIT = 0;

   reg [LEN-1:0] 	 data = INIT;
   assign o_q = data[0];
   assign o_par = data[LEN-1:1];
   always @(posedge clk)
     if (i_en)
       data <= {i_d, data[LEN-1:1]};
endmodule
