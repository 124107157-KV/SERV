`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2025 02:46:14
// Design Name: 
// Module Name: stochastic_relu
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

module stochastic_relu #(
    parameter WIDTH = 8,
    parameter STOCHASTIC_BITS = 256,
    parameter LFSR_WIDTH = 16
)(
    input clk,
    input reset,
    input signed [4*WIDTH-1:0] data_in, // Input from neuron (e.g., 32-bit signed)
    input enable,
    output reg signed [2*WIDTH-1:0] data_out, // Output to next layer (e.g., 16-bit signed)
    output reg relu_done
);

    // Internal signals for conversion and stochastic processing
    wire [STOCHASTIC_BITS-1:0] stochastic_data_in;
    wire [STOCHASTIC_BITS-1:0] stochastic_relu_out;
    wire [LFSR_WIDTH-1:0] lfsr_random_val; // Random value from LFSR for serv_stoc_alu
    wire lfsr_random_bit; // Random bit from LFSR for serv_stoc_alu
    wire bsc_conversion_done;
    wire sbc_conversion_done;

    // State machine for the ReLU process
    reg [1:0] current_state, next_state;

    // --- State Definitions ---
    localparam STATE_IDLE = 2'b00,
               STATE_BSC  = 2'b01,
               STATE_STOC_ALU = 2'b10,
               STATE_SBC  = 2'b11;

    // --- Data preparation for Unipolar BSC ---
    wire [4*WIDTH-1:0] bsc_input_unsigned;
    assign bsc_input_unsigned = (data_in > 0) ? data_in : 0;

    // --- Binary-to-Stochastic Converter (BSC) ---
    binary_to_stochastic_converter #(
        .BINARY_WIDTH(4*WIDTH), 
        .STOCHASTIC_BITS(STOCHASTIC_BITS),
        .LFSR_WIDTH(LFSR_WIDTH)
    ) bsc_inst (
        .clk(clk),
        .reset(reset),
        .binary_in(bsc_input_unsigned),
        .enable(current_state == STATE_BSC),
        .stochastic_out(stochastic_data_in),
        .conversion_done(bsc_conversion_done)
    );

    // --- LFSR for stochastic operations ---
    lfsr_rng #(.LFSR_WIDTH(LFSR_WIDTH)) stoc_alu_lfsr_inst (
        .clk(clk),
        .reset(reset),
        .random_out(lfsr_random_bit),
        .random_value(lfsr_random_val)
    );

    // Stochastic ReLU operation simulation
    reg [STOCHASTIC_BITS-1:0] stoc_alu_relu_internal_out;
    reg [$clog2(STOCHASTIC_BITS)-1:0] stoc_alu_bit_idx;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            stoc_alu_relu_internal_out <= 0;
            stoc_alu_bit_idx <= 0;
        end else if (current_state == STATE_STOC_ALU) begin
            if (stoc_alu_bit_idx < STOCHASTIC_BITS) begin
                stoc_alu_relu_internal_out[stoc_alu_bit_idx] <= stochastic_data_in[stoc_alu_bit_idx];
                stoc_alu_bit_idx <= stoc_alu_bit_idx + 1;
            end
        end else if (current_state == STATE_SBC) begin
            stoc_alu_bit_idx <= 0; // Reset for next operation
        end
    end
    assign stochastic_relu_out = stoc_alu_relu_internal_out;

    // --- Stochastic-to-Binary Converter (SBC) ---
    stochastic_to_binary_converter #(
        .STOCHASTIC_BITS(STOCHASTIC_BITS),
        .BINARY_WIDTH(2*WIDTH) 
    ) sbc_inst (
        .clk(clk),
        .reset(reset),
        .stochastic_in(stochastic_relu_out),
        .enable(current_state == STATE_SBC),
        .binary_out(data_out), 
        .conversion_done(sbc_conversion_done)
    );

    // --- State Machine Logic ---
    always @(*) begin
        next_state = current_state;
        relu_done = 0; // Default
        case (current_state)
            STATE_IDLE: begin
                if (enable) next_state = STATE_BSC;
            end
            STATE_BSC: begin
                if (bsc_conversion_done) next_state = STATE_STOC_ALU;
            end
            STATE_STOC_ALU: begin
                if (stoc_alu_bit_idx == STOCHASTIC_BITS) next_state = STATE_SBC;
            end
            STATE_SBC: begin
                if (sbc_conversion_done) begin
                    next_state = STATE_IDLE;
                    relu_done = 1; // Signal completion
                end
            end
        endcase
    end

    // --- State update process ---
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            current_state <= STATE_IDLE;
            data_out <= 0;
            relu_done <= 0;
        end else begin
            current_state <= next_state;
        end
    end

endmodule