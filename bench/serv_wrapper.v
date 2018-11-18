`default_nettype none
module serv_wrapper
(
 input wire  wb_clk,
 output wire q);

//   parameter memfile = "hellomin.hex";
   parameter memfile = "bitbang.hex";

   reg [4:0] rst_reg = 5'b11111;

   always @(posedge wb_clk)
     rst_reg <= {1'b0, rst_reg[4:1]};

   wire      wb_rst = rst_reg[0];

   wire 	timer_irq;

`include "wb_intercon.vh"

   localparam MEMORY_SIZE = 2048*4;

`ifndef SYNTHESIS
//synthesis translate_off
   reg [1023:0] firmware_file;
   initial
     if ($value$plusargs("firmware=%s", firmware_file)) begin
	$display("Loading RAM from %0s", firmware_file);
	$readmemh(firmware_file, ram.ram0.mem);
     end
//synthesis translate_on
`endif

   wb_ram
     #(
`ifdef SYNTHESIS
.memfile (memfile),
`endif
       .depth (MEMORY_SIZE))
   ram
     (// Wishbone interface
      .wb_clk_i (wb_clk),
      .wb_rst_i (wb_rst),
      .wb_adr_i (wb_m2s_mem_adr[$clog2(MEMORY_SIZE)-1:0]),
      .wb_stb_i (wb_m2s_mem_stb),
      .wb_cyc_i (wb_m2s_mem_cyc),
      .wb_cti_i (wb_m2s_mem_cti),
      .wb_bte_i (wb_m2s_mem_bte),
      .wb_we_i  (wb_m2s_mem_we) ,
      .wb_sel_i (wb_m2s_mem_sel),
      .wb_dat_i (wb_m2s_mem_dat),
      .wb_dat_o (wb_s2m_mem_dat),
      .wb_ack_o (wb_s2m_mem_ack),
      .wb_err_o ());

   testprint testprint
     (
      .i_wb_clk   (wb_clk),
      .i_wb_dat   (wb_m2s_testprint_dat),
      .i_wb_we    (wb_m2s_testprint_we),
      .i_wb_cyc   (wb_m2s_testprint_cyc),
      .i_wb_stb   (wb_m2s_testprint_stb),
      .o_wb_ack   (wb_s2m_testprint_ack));

   assign wb_s2m_testprint_dat = 32'h0;

   testhalt testhalt
     (
      .i_wb_clk   (wb_clk),
      .i_wb_dat   (wb_m2s_testhalt_dat),
      .i_wb_we    (wb_m2s_testhalt_we),
      .i_wb_cyc   (wb_m2s_testhalt_cyc),
      .i_wb_stb   (wb_m2s_testhalt_stb),
      .o_wb_ack   (wb_s2m_testhalt_ack));

   assign wb_s2m_testhalt_dat = 32'h0;

   riscv_timer riscv_timer
     (.i_clk    (wb_clk),
      .o_irq    (timer_irq),
      .i_wb_adr (wb_m2s_timer_adr),
      .i_wb_stb (wb_m2s_timer_stb),
      .i_wb_cyc (wb_m2s_timer_cyc),
      .i_wb_we  (wb_m2s_timer_we) ,
      .i_wb_sel (wb_m2s_timer_sel),
      .i_wb_dat (wb_m2s_timer_dat),
      .o_wb_dat (wb_s2m_timer_dat),
      .o_wb_ack (wb_s2m_timer_ack));

   wb_gpio gpio
     (.i_wb_clk (wb_clk),
      .i_wb_dat (wb_m2s_gpio_dat[0]),
      .i_wb_cyc (wb_m2s_gpio_cyc),
      .o_wb_ack (wb_s2m_gpio_ack),
      .o_gpio   (q));

   assign wb_s2m_gpio_dat = 32'h0;

   serv_top
     #(.RESET_PC (32'h0000_0000))
   cpu
     (
      .clk      (wb_clk),
      .i_rst    (wb_rst),
      .o_ibus_adr   (wb_m2s_cpu_ibus_adr),
      .o_ibus_cyc   (wb_m2s_cpu_ibus_cyc),
      .o_ibus_stb   (wb_m2s_cpu_ibus_stb),
      .i_ibus_rdt   (wb_s2m_cpu_ibus_dat),
      .i_ibus_ack   (wb_s2m_cpu_ibus_ack),
      .o_dbus_adr   (wb_m2s_cpu_dbus_adr),
      .o_dbus_dat   (wb_m2s_cpu_dbus_dat),
      .o_dbus_sel   (wb_m2s_cpu_dbus_sel),
      .o_dbus_we    (wb_m2s_cpu_dbus_we),
      .o_dbus_cyc   (wb_m2s_cpu_dbus_cyc),
      .o_dbus_stb   (wb_m2s_cpu_dbus_stb),
      .i_dbus_rdt   (wb_s2m_cpu_dbus_dat),
      .i_dbus_ack   (wb_s2m_cpu_dbus_ack));

   assign wb_m2s_cpu_ibus_dat = 32'd0;
   assign wb_m2s_cpu_ibus_we = 1'b0;
   assign wb_m2s_cpu_ibus_sel = 4'b1111;
   assign wb_m2s_cpu_ibus_cti = 3'b000;
   assign wb_m2s_cpu_ibus_bte = 2'b00;

endmodule
