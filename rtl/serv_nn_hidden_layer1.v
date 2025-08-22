`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.08.2025 16:03:41
// Design Name: 
// Module Name: serv_nn_hidden_layer1
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////


// File: serv_nn_hidden_layer1.v
`timescale 1ns / 1ps
module serv_nn_hidden_layer1 #(
    parameter FIXED_POINT_BITS = 1,
    parameter W = FIXED_POINT_BITS,
    parameter B = W - 1,
    parameter INPUT_DIM = 8,
    parameter OUTPUT_DIM = 6
)(
    input wire clk,
    input wire rst,
    input wire layer_start,

    // === ALU Interface ===
    output reg [4:0] o_alu_ctrl_bus,
    output reg [2:0] o_alu_cmp_ctrl_bus,
    output reg [(4*W)-1:0] o_alu_data_bus,
    output reg o_alu_en,
    input wire [B:0] i_alu_rd,
    input wire i_alu_cmp,

    input wire i_cnt_en,
    input wire i_cnt0,
    input wire i_cnt_done,
    input wire i_rf_ready,

    // === Layer I/O ===
    input wire [INPUT_DIM*FIXED_POINT_BITS-1:0] nn_input_vector,
    output wire [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_relu,
    output wire [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_sigmoid,
    output wire layer_done,
    output wire layer_ready
);

    // FSM States (identical to original)
    localparam [2:0]
        IDLE = 3'd0,
        MAC_STOC = 3'd1,
        ACCUM_STOC = 3'd2,
        ADD_BIAS_STD = 3'd3,
        ACTV_SIGMOID_STOC = 3'd4,
        ACTV_RELU_STD = 3'd5,
        NEXT_NEURON = 3'd6;

    // === Register declarations ===
    reg [2:0] nn_state, nn_next;
    reg [2:0] out_idx, out_idx_next;       // 6 outputs (0-5)
    reg [2:0] in_idx, in_idx_next;         // 8 inputs (0-7)
    reg [W+3-1:0] accum_std, accum_std_next;    // Larger accumulator for more inputs
    reg [W+3-1:0] accum_stoc, accum_stoc_next;
    reg [B:0] alu_res_reg, alu_res_reg_next;

    // Weight & bias arrays (8x6 matrix)
    reg [W-1:0] weights_arr [0:OUTPUT_DIM-1][0:INPUT_DIM-1];
    reg [W-1:0] bias_arr [0:OUTPUT_DIM-1];
    
    reg [OUTPUT_DIM*FIXED_POINT_BITS - 1:0] nn_output_vector_relu_i;
    reg [OUTPUT_DIM*FIXED_POINT_BITS - 1:0] nn_output_vector_sigmoid_i;
    reg layer_done_i;
    reg layer_ready_i;
    
    assign nn_output_vector_relu = nn_output_vector_relu_i;
    assign nn_output_vector_sigmoid = nn_output_vector_sigmoid_i;
    assign layer_done = layer_done_i;
    assign layer_ready = layer_ready_i;

    integer i, j;
    // Initialize weights & biases on reset
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            // Hidden layer 1 weights initialization (8x6 matrix)
            weights_arr[0][0] = 8'd95;  weights_arr[0][1] = 8'd145; weights_arr[0][2] = 8'd175; weights_arr[0][3] = 8'd65;
            weights_arr[0][4] = 8'd125; weights_arr[0][5] = 8'd185; weights_arr[0][6] = 8'd105; weights_arr[0][7] = 8'd155;
            
            weights_arr[1][0] = 8'd135; weights_arr[1][1] = 8'd85;  weights_arr[1][2] = 8'd195; weights_arr[1][3] = 8'd115;
            weights_arr[1][4] = 8'd165; weights_arr[1][5] = 8'd75;  weights_arr[1][6] = 8'd205; weights_arr[1][7] = 8'd95;
            
            weights_arr[2][0] = 8'd115; weights_arr[2][1] = 8'd175; weights_arr[2][2] = 8'd85;  weights_arr[2][3] = 8'd145;
            weights_arr[2][4] = 8'd195; weights_arr[2][5] = 8'd105; weights_arr[2][6] = 8'd165; weights_arr[2][7] = 8'd125;
            
            weights_arr[3][0] = 8'd155; weights_arr[3][1] = 8'd105; weights_arr[3][2] = 8'd135; weights_arr[3][3] = 8'd185;
            weights_arr[3][4] = 8'd75;  weights_arr[3][5] = 8'd165; weights_arr[3][6] = 8'd115; weights_arr[3][7] = 8'd195;
            
            weights_arr[4][0] = 8'd185; weights_arr[4][1] = 8'd125; weights_arr[4][2] = 8'd165; weights_arr[4][3] = 8'd95;
            weights_arr[4][4] = 8'd205; weights_arr[4][5] = 8'd85;  weights_arr[4][6] = 8'd145; weights_arr[4][7] = 8'd115;
            
            weights_arr[5][0] = 8'd105; weights_arr[5][1] = 8'd195; weights_arr[5][2] = 8'd115; weights_arr[5][3] = 8'd175;
            weights_arr[5][4] = 8'd135; weights_arr[5][5] = 8'd155; weights_arr[5][6] = 8'd85;  weights_arr[5][7] = 8'd165;

            // Hidden layer 1 biases
            bias_arr[0] = 8'd18; bias_arr[1] = 8'd22; bias_arr[2] = 8'd14;
            bias_arr[3] = 8'd26; bias_arr[4] = 8'd16; bias_arr[5] = 8'd20;

            // Reset outputs
            nn_output_vector_relu_i <= 0;
            nn_output_vector_sigmoid_i <= 0;
            layer_done_i <= 0;
            layer_ready_i <= 1;
        end
    end

    // === Combinational Next-State + Outputs (same logic as original) ===
    always @(*) begin
        // Default assignments
        nn_next = nn_state;
        out_idx_next = out_idx;
        in_idx_next = in_idx;
        accum_std_next = accum_std;
        accum_stoc_next = accum_stoc;
        alu_res_reg_next = alu_res_reg;
        o_alu_en = 1'b0;
        o_alu_ctrl_bus = 5'b0;
        o_alu_cmp_ctrl_bus = 3'b0;
        o_alu_data_bus = {4*W{1'b0}};
        layer_done_i = 1'b0;
        layer_ready_i = (nn_state == IDLE);

        case (nn_state)
            IDLE: begin
                if (layer_start) begin
                    out_idx_next = 0;
                    accum_std_next = 0;
                    accum_stoc_next = 0;
                    in_idx_next = 0;
                    nn_next = MAC_STOC;
                end
            end

            MAC_STOC: begin
                // Issue stochastic multiply for this input
                o_alu_en = 1'b1;
                o_alu_ctrl_bus = {3'b011, 2'b00}; // Stochastic multiplication
                o_alu_data_bus = {
                    nn_input_vector[in_idx*W +: W],
                    weights_arr[out_idx][in_idx],
                    {2*W{1'b0}}
                };
                if (i_cnt_done && i_rf_ready) begin
                    alu_res_reg_next = i_alu_rd;
                    accum_stoc_next = accum_stoc + i_alu_rd;
                    nn_next = ACCUM_STOC;
                end
            end

            ACCUM_STOC: begin
                accum_std_next = accum_std + alu_res_reg;
                if (in_idx == INPUT_DIM-1) begin
                    nn_next = ADD_BIAS_STD;
                end else begin
                    in_idx_next = in_idx + 1;
                    nn_next = MAC_STOC;
                end
            end

            ADD_BIAS_STD: begin
                o_alu_en = 1'b1;
                o_alu_ctrl_bus = {3'b000, 2'b00}; // addition
                o_alu_data_bus = {
                    accum_std[W-1:0],
                    bias_arr[out_idx],
                    {2*W{1'b0}}
                };
                if (i_cnt_done && i_rf_ready) begin
                    accum_std_next = accum_std + bias_arr[out_idx];
                    nn_next = ACTV_SIGMOID_STOC;
                end
            end

            ACTV_SIGMOID_STOC: begin
                o_alu_en = 1'b1;
                o_alu_ctrl_bus = {3'b100, 2'b00}; // sigmoid
                o_alu_data_bus = {
                    accum_stoc[W-1:0],
                    {3*W{1'b0}}
                };
                if (i_cnt_done && i_rf_ready) begin
                    nn_output_vector_sigmoid_i[out_idx*W +: W] = i_alu_rd;
                    nn_next = ACTV_RELU_STD;
                end
            end

            ACTV_RELU_STD: begin
                o_alu_en = 1'b1;
                o_alu_cmp_ctrl_bus = {1'b0, 1'b1, 1'b0}; // comparison
                o_alu_ctrl_bus = {3'b010, 2'b00}; // SLT
                o_alu_data_bus = {
                    accum_std[W-1:0],
                    {W{1'b0}},
                    {2*W{1'b0}}
                };
                if (i_cnt_done && i_rf_ready) begin
                    if (i_alu_cmp) begin
                        nn_output_vector_relu_i[out_idx*W +: W] = {W{1'b0}};
                    end else begin
                        nn_output_vector_relu_i[out_idx*W +: W] = accum_std[W-1:0];
                    end
                    nn_next = NEXT_NEURON;
                end
            end

            NEXT_NEURON: begin
                if (out_idx == OUTPUT_DIM-1) begin
                    layer_done_i = 1'b1;
                    nn_next = IDLE;
                end else begin
                    out_idx_next = out_idx + 1;
                    in_idx_next = 0;
                    accum_std_next = 0;
                    accum_stoc_next = 0;
                    nn_next = MAC_STOC;
                end
            end
        endcase
    end

    // === Sequential State Update ===
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            nn_state <= IDLE;
            out_idx <= 0;
            in_idx <= 0;
            accum_std <= 0;
            accum_stoc <= 0;
            alu_res_reg <= 0;
        end else begin
            nn_state <= nn_next;
            out_idx <= out_idx_next;
            in_idx <= in_idx_next;
            accum_std <= accum_std_next;
            accum_stoc <= accum_stoc_next;
            alu_res_reg <= alu_res_reg_next;
        end
    end

endmodule
