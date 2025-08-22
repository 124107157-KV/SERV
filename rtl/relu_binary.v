// FileName: relu_binary.v
`timescale 1ns / 1ps

/**********
* Relu activation function implementation for binary data.
*
* Parameters: WIDTH => The width used in the NN
*
* Inputs: data_in => in element, 
*
* Outputs: data_out => relu output (ie data_in if >0, else 0)
* 
***********/

module relu_binary #(parameter WIDTH = 8)(
    input signed [4*WIDTH-1:0] data_in,
    output signed [2*WIDTH-1:0] data_out
    );
    
    wire signed [4*WIDTH-1:0] temp;
    
    assign temp = (data_in > 0)? data_in:0; //Take data_in if > 0, 0 else
    assign data_out = temp >> 8; //Rescale element and store into data_out
    
endmodule