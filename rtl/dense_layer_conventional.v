// FileName: dense_layer_conventional.v
`timescale 1ns / 1ps

/**********
* Dense layer implementation using conventional neurons.
*
* Parameters: NEURON_NB => The # of neurons
*             IN_SIZE => The input vector size
*             WIDTH => The width of the weights and biases
*
* Inputs: clk => clock signal, layer_en => enable signal, 
*         reset => active high sync reset signal, in_data => in vector, 
*         weights => neurons weights, biases => neurons biases
*
* Outputs: neuron_out => dense layer output
*          layer_done => done signal
* 
***********/

module dense_layer_conventional # (parameter NEURON_NB=32, IN_SIZE=196, WIDTH=8)(
    input clk,
    input layer_en,
    input reset,
    input signed[2*WIDTH-1:0] in_data [0:IN_SIZE-1],
    input signed[WIDTH-1:0] weights [0:NEURON_NB-1][0:IN_SIZE-1],
    input signed[WIDTH-1:0] biases [0:NEURON_NB-1],
    output signed[4*WIDTH-1:0] neuron_out [0:NEURON_NB-1],
    output layer_done
    );
    
    // Array to track completion of each neuron
    wire [0:NEURON_NB-1] neuron_done_signals;
    
    // Instantiate NEURON_NB neuron_conventional submodules
    genvar i;
    generate
        for (i = 0; i < NEURON_NB; i = i + 1) begin : gen_neurons
            neuron_conventional #(.IN_SIZE(IN_SIZE), .WIDTH(WIDTH)) 
            dense_neuron_inst (
                .clk(clk), 
                .en(layer_en), 
                .reset(reset), 
                .in_data(in_data), 
                .weight(weights[i]), // Pass weights for current neuron
                .bias(biases[i]),    // Pass bias for current neuron
                .neuron_out(neuron_out[i]), 
                .neuron_done(neuron_done_signals[i])
            );
        end
    endgenerate
    
    // The layer is done when all individual neurons are done
    assign layer_done = &neuron_done_signals;

endmodule