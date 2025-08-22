`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 18.08.2025 15:20:01
// Design Name: 
// Module Name: serv_multi_layer_nn_top
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


// File: serv_multi_layer_nn_top.v
`timescale 1ns / 1ps

module serv_multi_layer_nn_top #(
    parameter FIXED_POINT_BITS = 1,
    parameter W = FIXED_POINT_BITS,
    parameter B = W - 1,
    parameter INPUT_DIM = 4,
    parameter HIDDEN1_DIM = 8,
    parameter HIDDEN2_DIM = 6,
    parameter OUTPUT_DIM = 3,
    parameter NUM_LAYERS = 4
)(
    input wire clk,
    input wire rst,
    
    // === ALU Interface to serv_top ===
    output reg [4:0] o_alu_ctrl_bus,
    output reg [2:0] o_alu_cmp_ctrl_bus,
    output reg [(4*W)-1:0] o_alu_data_bus,
    output reg o_alu_en,
    input wire [B:0] i_alu_rd,
    input wire i_alu_cmp,
    
    // CPU Interface
    input wire i_cnt_en,
    input wire i_cnt0,
    input wire i_cnt_done,
    input wire i_rf_ready,
    
    // === Network I/O ===
    input wire [INPUT_DIM*FIXED_POINT_BITS-1:0] nn_input_vector,
    input wire nn_start,
    output reg [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_relu,
    output reg [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_sigmoid,
    output reg [OUTPUT_DIM*FIXED_POINT_BITS-1:0] nn_output_vector_tanh,
    output reg nn_done,
    output reg nn_ready,
    output reg [7:0] nn_status
);

    // Global FSM States
    localparam [3:0]
        GLOBAL_IDLE = 4'd0,
        GLOBAL_INIT = 4'd1,
        LAYER_0_PROC = 4'd2,
        LAYER_1_PROC = 4'd3,
        LAYER_2_PROC = 4'd4,
        LAYER_3_PROC = 4'd5,
        GLOBAL_DONE = 4'd6,
        GLOBAL_ERROR = 4'd7;

    // State registers
    reg [3:0] global_state, global_next_state;
    reg [2:0] current_layer;
    reg [15:0] cycle_counter;

    // Inter-layer data buffers (following original structure)
    wire [HIDDEN1_DIM*FIXED_POINT_BITS-1:0] layer0_output_relu;
    wire [HIDDEN1_DIM*FIXED_POINT_BITS-1:0] layer0_output_sigmoid;
    
    wire [HIDDEN2_DIM*FIXED_POINT_BITS-1:0] layer1_output_relu;
    wire [HIDDEN2_DIM*FIXED_POINT_BITS-1:0] layer1_output_sigmoid;
    
    wire [OUTPUT_DIM*FIXED_POINT_BITS-1:0] layer2_output_relu;
    wire [OUTPUT_DIM*FIXED_POINT_BITS-1:0] layer2_output_sigmoid;
    wire [OUTPUT_DIM*FIXED_POINT_BITS-1:0] layer2_output_tanh;

    // Layer control signals
    wire [4:0] layer0_alu_ctrl_bus, layer1_alu_ctrl_bus, layer2_alu_ctrl_bus;
    wire [2:0] layer0_alu_cmp_ctrl_bus, layer1_alu_cmp_ctrl_bus, layer2_alu_cmp_ctrl_bus;
    wire [(4*W)-1:0] layer0_alu_data_bus, layer1_alu_data_bus, layer2_alu_data_bus;
    wire layer0_alu_en, layer1_alu_en, layer2_alu_en;
    wire layer0_done, layer1_done, layer2_done;
    wire layer0_ready, layer1_ready, layer2_ready;
    
    reg layer0_start, layer1_start, layer2_start;

    // Layer 0: Input Layer (4→8 neurons) - ReLU + Sigmoid
    serv_nn_input_layer #(
        .FIXED_POINT_BITS(FIXED_POINT_BITS),
        .W(W),
        .INPUT_DIM(INPUT_DIM),
        .OUTPUT_DIM(HIDDEN1_DIM)
    ) input_layer (
        .clk(clk),
        .rst(rst),
        .layer_start(layer0_start),
        .o_alu_ctrl_bus(layer0_alu_ctrl_bus),
        .o_alu_cmp_ctrl_bus(layer0_alu_cmp_ctrl_bus),
        .o_alu_data_bus(layer0_alu_data_bus),
        .o_alu_en(layer0_alu_en),
        .i_alu_rd(i_alu_rd),
        .i_alu_cmp(i_alu_cmp),
        .i_cnt_en(i_cnt_en),
        .i_cnt0(i_cnt0),
        .i_cnt_done(i_cnt_done),
        .i_rf_ready(i_rf_ready),
        .nn_input_vector(nn_input_vector),
        .nn_output_vector_relu(layer0_output_relu),
        .nn_output_vector_sigmoid(layer0_output_sigmoid),
        .layer_done(layer0_done),
        .layer_ready(layer0_ready)
    );

    // Layer 1: Hidden Layer 1 (8→6 neurons) - ReLU + Sigmoid
    serv_nn_hidden_layer1 #(
        .FIXED_POINT_BITS(FIXED_POINT_BITS),
        .W(W),
        .INPUT_DIM(HIDDEN1_DIM),
        .OUTPUT_DIM(HIDDEN2_DIM)
    ) hidden_layer1 (
        .clk(clk),
        .rst(rst),
        .layer_start(layer1_start),
        .o_alu_ctrl_bus(layer1_alu_ctrl_bus),
        .o_alu_cmp_ctrl_bus(layer1_alu_cmp_ctrl_bus),
        .o_alu_data_bus(layer1_alu_data_bus),
        .o_alu_en(layer1_alu_en),
        .i_alu_rd(i_alu_rd),
        .i_alu_cmp(i_alu_cmp),
        .i_cnt_en(i_cnt_en),
        .i_cnt0(i_cnt0),
        .i_cnt_done(i_cnt_done),
        .i_rf_ready(i_rf_ready),
        .nn_input_vector(layer0_output_sigmoid), // Use sigmoid output from previous layer
        .nn_output_vector_relu(layer1_output_relu),
        .nn_output_vector_sigmoid(layer1_output_sigmoid),
        .layer_done(layer1_done),
        .layer_ready(layer1_ready)
    );

    // Layer 2: Output Layer (6→3 neurons) - ReLU + Sigmoid + Tanh
    serv_nn_output_layer #(
        .FIXED_POINT_BITS(FIXED_POINT_BITS),
        .W(W),
        .INPUT_DIM(HIDDEN2_DIM),
        .OUTPUT_DIM(OUTPUT_DIM)
    ) output_layer (
        .clk(clk),
        .rst(rst),
        .layer_start(layer2_start),
        .o_alu_ctrl_bus(layer2_alu_ctrl_bus),
        .o_alu_cmp_ctrl_bus(layer2_alu_cmp_ctrl_bus),
        .o_alu_data_bus(layer2_alu_data_bus),
        .o_alu_en(layer2_alu_en),
        .i_alu_rd(i_alu_rd),
        .i_alu_cmp(i_alu_cmp),
        .i_cnt_en(i_cnt_en),
        .i_cnt0(i_cnt0),
        .i_cnt_done(i_cnt_done),
        .i_rf_ready(i_rf_ready),
        .nn_input_vector(layer1_output_sigmoid), // Use sigmoid output from previous layer
        .nn_output_vector_relu(layer2_output_relu),
        .nn_output_vector_sigmoid(layer2_output_sigmoid),
        .nn_output_vector_tanh(layer2_output_tanh),
        .layer_done(layer2_done),
        .layer_ready(layer2_ready)
    );

    // Global FSM Control Logic
    always @(*) begin
        // Default assignments
        global_next_state = global_state;
        o_alu_en = 1'b0;
        o_alu_ctrl_bus = 5'b0;
        o_alu_cmp_ctrl_bus = 3'b0;
        o_alu_data_bus = {4*W{1'b0}};
        nn_done = 1'b0;
        nn_ready = (global_state == GLOBAL_IDLE);
        
        // Clear layer start signals
        layer0_start = 1'b0;
        layer1_start = 1'b0;
        layer2_start = 1'b0;

        case (global_state)
            GLOBAL_IDLE: begin
                nn_status = 8'h00; // Ready
                if (nn_start) begin
                    global_next_state = GLOBAL_INIT;
                end
            end

            GLOBAL_INIT: begin
                nn_status = 8'h01; // Initializing
                global_next_state = LAYER_0_PROC;
            end

            LAYER_0_PROC: begin
                nn_status = 8'h10; // Processing Layer 0
                layer0_start = 1'b1;
                // Route Layer 0 ALU requests
                o_alu_en = layer0_alu_en;
                o_alu_ctrl_bus = layer0_alu_ctrl_bus;
                o_alu_cmp_ctrl_bus = layer0_alu_cmp_ctrl_bus;
                o_alu_data_bus = layer0_alu_data_bus;
                
                if (layer0_done) begin
                    global_next_state = LAYER_1_PROC;
                end
            end

            LAYER_1_PROC: begin
                nn_status = 8'h20; // Processing Layer 1
                layer1_start = 1'b1;
                // Route Layer 1 ALU requests
                o_alu_en = layer1_alu_en;
                o_alu_ctrl_bus = layer1_alu_ctrl_bus;
                o_alu_cmp_ctrl_bus = layer1_alu_cmp_ctrl_bus;
                o_alu_data_bus = layer1_alu_data_bus;
                
                if (layer1_done) begin
                    global_next_state = LAYER_2_PROC;
                end
            end

            LAYER_2_PROC: begin
                nn_status = 8'h30; // Processing Layer 2
                layer2_start = 1'b1;
                // Route Layer 2 ALU requests
                o_alu_en = layer2_alu_en;
                o_alu_ctrl_bus = layer2_alu_ctrl_bus;
                o_alu_cmp_ctrl_bus = layer2_alu_cmp_ctrl_bus;
                o_alu_data_bus = layer2_alu_data_bus;
                
                if (layer2_done) begin
                    global_next_state = GLOBAL_DONE;
                end
            end

            GLOBAL_DONE: begin
                nn_status = 8'hFF; // Done
                nn_done = 1'b1;
                global_next_state = GLOBAL_IDLE;
            end

            GLOBAL_ERROR: begin
                nn_status = 8'hEE; // Error
                global_next_state = GLOBAL_IDLE;
            end

            default: begin
                global_next_state = GLOBAL_IDLE;
            end
        endcase
    end

    // Sequential state update
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            global_state <= GLOBAL_IDLE;
            current_layer <= 0;
            cycle_counter <= 0;
            nn_output_vector_relu <= 0;
            nn_output_vector_sigmoid <= 0;
            nn_output_vector_tanh <= 0;
        end else begin
            global_state <= global_next_state;
            cycle_counter <= cycle_counter + 1;
            
            // Update current layer tracking
            case (global_state)
                LAYER_0_PROC: current_layer <= 0;
                LAYER_1_PROC: current_layer <= 1;
                LAYER_2_PROC: current_layer <= 2;
                default: current_layer <= 0;
            endcase
            
            // Capture final outputs
            if (global_state == GLOBAL_DONE) begin
                nn_output_vector_relu <= layer2_output_relu;
                nn_output_vector_sigmoid <= layer2_output_sigmoid;
                nn_output_vector_tanh <= layer2_output_tanh;
            end
        end
    end

endmodule