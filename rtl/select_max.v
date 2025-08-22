// FileName: select_max.v
`timescale 1ns / 1ps

/**********
* Module to select the maximum between last layer outputs
*
* Inputs: clk => clock signal, enable => enable signal,
*         reset => active high sync reset, in_data => input array, 
*
* Outputs: digit => Index of max element (ie handwritten digit)
*          layer_done => done signal
* 
***********/

module select_max # (parameter NEURON_NB=10, WIDTH=8)(
    input clk,
    input enable,
    input reset,
    input signed[2*WIDTH-1:0] in_data [0:NEURON_NB-1],
    output [WIDTH-1:0] digit,
    output layer_done
    );
    
    integer i = 0;
    reg signed [2*WIDTH-1:0] max = 0;
    reg signed [WIDTH-1:0] index = 0;
    reg done = 0;
    
    always @(posedge clk) begin
        if(reset) begin
            done <= 0;
            i <= 0;
            max <= 0;
            index <= 0;
        end
        else if(enable) begin
            if (i < NEURON_NB) begin // Iterate through all neurons
                if (in_data[i] >= max) begin //Update maximum and max index
                    max <= in_data[i]; 
                    index <= i;
                end
                i <= i + 1;
                done <= 0;
            end else begin
                done <= 1; // All elements scanned
                i <= 0; // Reset for next enable cycle
            end
        end
    end
    
    assign digit = index;
    assign layer_done = done;
    
endmodule