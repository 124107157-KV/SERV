// FileName: neuron_conventional.v
`timescale 1ns / 1ps

/**********
* Single neuron implementation using conventional binary arithmetic.
* This module's operations (multiplication, addition) would map to the
* conventional SERV ALU (serv_alu) if executed on the SERV core.
*
* Parameters: IN_SIZE => The input vector size
*             WIDTH => The width of the weights and biases
*
* Inputs: clk => clock signal, en => enable signal,
*         in_data => in vector, reset => active high sync reset
*         weights => neuron weights, bias => neuron bias
*
* Outputs: neuron_out => neuron value before activation
*          neuron_done => done signal
* 
***********/

module neuron_conventional #(parameter IN_SIZE=196, WIDTH = 8)(
    input clk,
    input en,
    input reset,
    input signed[2*WIDTH-1:0] in_data[0:IN_SIZE-1],
    input signed[WIDTH-1:0] weight[0:IN_SIZE-1],
    input signed[WIDTH-1:0] bias,
    output signed[4*WIDTH-1:0] neuron_out,
    output neuron_done
    );
    
    integer addr = 0;
    reg done = 0;
    
    reg signed [4*WIDTH-1:0] product_reg = 0; // Intermediate product
    reg signed [4*WIDTH-1:0] sum_out_reg = 0; // Accumulator for weighted sum
    
    always @(posedge clk) begin
        if(reset) begin 
            done <= 0;
            addr <= 0;
            product_reg <= 0;
            sum_out_reg <= 0;
        end
        else if(en) begin
            if(addr < IN_SIZE) begin // Iterate through all inputs
                // Conventional binary multiplication and addition
                product_reg <= in_data[addr] * weight[addr]; 
                sum_out_reg <= sum_out_reg + product_reg; 
                addr <= addr + 1;
                done <= 0; // Not done until all inputs processed
            end
            else begin // All inputs processed, add bias and signal done
                sum_out_reg <= sum_out_reg + bias; // Add bias
                done <= 1;
                addr <= 0; // Reset address for next enable cycle
            end
        end
    end
    
    assign neuron_out = sum_out_reg; // Output the final weighted sum + bias
    assign neuron_done = done;
    
endmodule