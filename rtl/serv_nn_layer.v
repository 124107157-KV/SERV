`timescale 1ns / 1ps
`default_nettype none

module serv_nn_layer #(
    parameter FIXED_POINT_BITS = 8,
    parameter W  = FIXED_POINT_BITS,
    parameter B  = W - 1,
    parameter INPUT_DIM  = 4,
    parameter OUTPUT_DIM = 2
)(
    input  wire                       clk,
    input  wire                       rst,

    // === ALU Interface to serv_top ===
    output reg  [4:0]  o_alu_ctrl_bus,
    output reg  [2:0]  o_alu_cmp_ctrl_bus,
    output reg [(4*W)-1:0] o_alu_data_bus,
    output reg         o_alu_en,
    input  wire [B:0]  i_alu_rd,
    input  wire        i_alu_cmp,

    input  wire        i_cnt_en,
    input  wire        i_cnt0,
    input  wire        i_cnt_done,
    input  wire        i_rf_ready,

    // === NN I/O ===
    input  wire [INPUT_DIM*FIXED_POINT_BITS-1:0] nn_input_vector,
    output reg [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_relu,
    output reg [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_sigmoid
);

    // FSM States
    localparam [2:0]
      IDLE               = 3'd0,
      MAC_STOC           = 3'd1,
      ACCUM_STOC         = 3'd2,
      ADD_BIAS_STD       = 3'd3,
      ACTV_SIGMOID_STOC  = 3'd4,
      ACTV_RELU_STD      = 3'd5,
      NEXT_NEURON        = 3'd6;

    // === register declarations ===
    reg [2:0]  nn_state,        nn_next;
    reg [1:0]  out_idx,         out_idx_next;
    reg [1:0]  in_idx,          in_idx_next;
    reg [W+2-1:0] accum_std,    accum_std_next;
    reg [W+2-1:0] accum_stoc,   accum_stoc_next;
    reg [B:0]  alu_res_reg,     alu_res_reg_next;

    // Weight & bias arrays
    reg [W-1:0] weights_arr [0:OUTPUT_DIM-1][0:INPUT_DIM-1];
    reg [W-1:0] bias_arr    [0:OUTPUT_DIM-1];

    integer i,j;
    // Initialize weights & biases on reset
    always @(posedge clk or posedge rst) begin
      if (rst) begin
        // Example hard-coded initialisation
        weights_arr[0][0]=8'd153; weights_arr[0][1]=8'd179;
        weights_arr[0][2]=8'd230; weights_arr[0][3]=8'd25;
        weights_arr[1][0]=8'd224; weights_arr[1][1]=8'd64;
        weights_arr[1][2]=8'd192; weights_arr[1][3]=8'd128;
        bias_arr[0] = 8'd10;
        bias_arr[1] = 8'd5;

        // Reset outputs
        nn_output_vector_relu    <= 0;
        nn_output_vector_sigmoid <= 0;
      end
    end

    // === Combinational Next-State + Outputs ===
    always @(*) begin
      // Default assignments
      nn_next         = nn_state;
      out_idx_next    = out_idx;
      in_idx_next     = in_idx;
      accum_std_next  = accum_std;
      accum_stoc_next = accum_stoc;
      alu_res_reg_next= alu_res_reg;
      o_alu_en        = 1'b0;
      o_alu_ctrl_bus  = 5'b0;
      o_alu_cmp_ctrl_bus = 3'b0;
      o_alu_data_bus  = {4*W{1'b0}};

      case (nn_state)
        IDLE: begin
          out_idx_next    = 0;
          accum_std_next  = 0;
          accum_stoc_next = 0;
          in_idx_next     = 0;
          nn_next         = MAC_STOC;
        end

        MAC_STOC: begin
          // Issue stochastic multiply (AND) for this input
          o_alu_en       = 1'b1;
          // rd_sel = 3'b011 (example code for mult), bool_op unused
          o_alu_ctrl_bus = {3'b011, 2'b00};
          o_alu_data_bus = {
            // i_rs1 = input
            nn_input_vector[in_idx*W +: W],
            // i_op_b = weight
            weights_arr[out_idx][in_idx],
            // buf & mat_elem unused
            {2*W{1'b0}}
          };
          if (i_cnt_done && i_rf_ready) begin
            alu_res_reg_next = i_alu_rd;
            accum_stoc_next  = accum_stoc + i_alu_rd;
            nn_next          = ACCUM_STOC;
          end
        end

        ACCUM_STOC: begin
          // Also accumulate in fixed domain
          accum_std_next = accum_std + alu_res_reg;
          // Advance to next input
          if (in_idx == INPUT_DIM-1) begin
            nn_next = ADD_BIAS_STD;
          end else begin
            in_idx_next = in_idx + 1;
            nn_next     = MAC_STOC;
          end
        end

        ADD_BIAS_STD: begin
          // Add bias using standard ALU
          o_alu_en       = 1'b1;
          o_alu_ctrl_bus = {3'b000, 2'b00}; // add
          o_alu_data_bus = {
            accum_std[W-1:0],       // i_rs1
            bias_arr[out_idx],      // i_op_b
            {2*W{1'b0}}
          };
          if (i_cnt_done && i_rf_ready) begin
            accum_std_next = accum_std + bias_arr[out_idx];
            nn_next = ACTV_SIGMOID_STOC;
          end
        end

        ACTV_SIGMOID_STOC: begin
          // Sigmoid via stochastic ALU
          o_alu_en       = 1'b1;
          // rd_sel = e.g. 3'b101 for sigmoid in your stoc_alu
          o_alu_ctrl_bus = {3'b101, 2'b00};
          o_alu_data_bus = {
            accum_stoc[W-1:0], // i_rs1
            {3*W{1'b0}}
          };
          if (i_cnt_done && i_rf_ready) begin
            nn_output_vector_sigmoid[out_idx*W +: W] = i_alu_rd;
            nn_next = ACTV_RELU_STD;
          end
        end

        ACTV_RELU_STD: begin
          // ReLU via standard comparator+mux
          o_alu_en         = 1'b1;
          o_alu_cmp_ctrl_bus = {1'b0,1'b1,1'b0}; // cmp_eq=1
          // Ask ALU to do SLT: rd_sel=3'b010
          o_alu_ctrl_bus  = {3'b010, 2'b00};
          o_alu_data_bus  = {
            accum_std[W-1:0], // i_rs1
            {W{1'b0}},
            {2*W{1'b0}}
          };
          if (i_cnt_done && i_rf_ready) begin
            if (i_alu_cmp) begin
              nn_output_vector_relu[out_idx*W +: W] = {W{1'b0}};
            end else begin
              nn_output_vector_relu[out_idx*W +: W] = accum_std[W-1:0];
            end
            nn_next = NEXT_NEURON;
          end
        end

        NEXT_NEURON: begin
          if (out_idx == OUTPUT_DIM-1) begin
            nn_next = IDLE; // done all neurons
          end else begin
            out_idx_next    = out_idx + 1;
            in_idx_next     = 0;
            accum_std_next  = 0;
            accum_stoc_next = 0;
            nn_next         = MAC_STOC;
          end
        end
      endcase
    end

    // === Sequential State Update ===
    always @(posedge clk or posedge rst) begin
      if (rst) begin
        nn_state       <= IDLE;
        out_idx        <= 0;
        in_idx         <= 0;
        accum_std      <= 0;
        accum_stoc     <= 0;
        alu_res_reg    <= 0;
      end else begin
        nn_state       <= nn_next;
        out_idx        <= out_idx_next;
        in_idx         <= in_idx_next;
        accum_std      <= accum_std_next;
        accum_stoc     <= accum_stoc_next;
        alu_res_reg    <= alu_res_reg_next;
      end
    end

endmodule