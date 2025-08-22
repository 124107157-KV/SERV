// FileName: neural_layer_dual_alu.v
`timescale 1ns / 1ps

/**********
* A configurable neural layer that uses conventional ALU for weighted sum
* and bias, and allows selection between binary and conceptual stochastic
* ALU for activation.
*
* Parameters: NEURON_NB => Number of neurons in this layer
*             IN_SIZE => Size of the input vector
*             DATA_WIDTH => Bit width for data (e.g., 8 for 8-bit values)
*             USE_STOCHASTIC_ACTIVATION => 1 to use stochastic_relu_placeholder, 0 for relu_binary
*
* Inputs: clk => clock signal
*         enable => Layer enable signal
*         reset => Active high synchronous reset
*         in_data => Input vector to the layer
*         weights => Weights for each neuron in this layer
*         biases => Biases for each neuron in this layer
*
* Outputs: layer_out => Output vector of the layer after activation
*          layer_done => Signal indicating layer computation is complete
* 
***********/

module neural_layer_dual_alu # (
    parameter NEURON_NB = 32,
    parameter IN_SIZE = 196,
    parameter DATA_WIDTH = 8,
    parameter USE_STOCHASTIC_ACTIVATION = 0 // 0 for binary ReLU, 1 for stochastic placeholder
)(
    input clk,
    input enable,
    input reset,
    input signed[2*DATA_WIDTH-1:0] in_data [0:IN_SIZE-1],
    input signed[DATA_WIDTH-1:0] weights [0:NEURON_NB-1][0:IN_SIZE-1],
    input signed[DATA_WIDTH-1:0] biases [0:NEURON_NB-1],
    output signed[2*DATA_WIDTH-1:0] layer_out [0:NEURON_NB-1],
    output layer_done
);

    // Internal wire for the output of the dense layer (before activation)
    wire signed[4*DATA_WIDTH-1:0] dense_output_pre_activation [0:NEURON_NB-1];
    wire dense_layer_done;

    // Instantiate the conventional dense layer for weighted sum and bias
    // This part implicitly uses the conventional SERV ALU (serv_alu)
    dense_layer_conventional # (
        .NEURON_NB(NEURON_NB),
        .IN_SIZE(IN_SIZE),
        .WIDTH(DATA_WIDTH)
    ) dense_layer_inst (
        .clk(clk),
        .layer_en(enable),
        .reset(reset),
        .in_data(in_data),
        .weights(weights),
        .biases(biases),
        .neuron_out(dense_output_pre_activation),
        .layer_done(dense_layer_done)
    );

    // Wires for activation outputs
    wire signed[2*DATA_WIDTH-1:0] activated_output [0:NEURON_NB-1];

    // Generate block to select the activation function
    genvar i;
    generate
        if (USE_STOCHASTIC_ACTIVATION == 1) begin : gen_stochastic_activation
            // Use stochastic_relu_placeholder for activation
            for (i = 0; i < NEURON_NB; i = i + 1) begin : stoc_relu_inst
                stochastic_relu #(.WIDTH(DATA_WIDTH)) 
                stoc_relu (
                    .clk(clk),
                    .data_in(dense_output_pre_activation[i]),
                    .data_out(activated_output[i])
                );
            end
        end else begin : gen_binary_activation
            // Use relu_binary for activation (default)
            for (i = 0; i < NEURON_NB; i = i + 1) begin : bin_relu_inst
                relu_binary #(.WIDTH(DATA_WIDTH)) 
                bin_relu (
                    .data_in(dense_output_pre_activation[i]),
                    .data_out(activated_output[i])
                );
            end
        end
    endgenerate

    // Output of this layer is the activated output
    assign layer_out = activated_output;
    assign layer_done = dense_layer_done; // Layer is done when dense computation is done

endmodule