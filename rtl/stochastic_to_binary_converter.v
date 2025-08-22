// FileName: stochastic_to_binary_converter.v
`timescale 1ns / 1ps

/**********
* Stochastic-to-Binary Converter (SBC) - Unipolar Encoding.
* Converts a unipolar stochastic bitstream back to a binary value.
* This is typically done by counting the number of '1's in the bitstream.
*
* Parameters: STOCHASTIC_BITS => Length of the input stochastic bitstream
*             BINARY_WIDTH => Bit width of the output binary value
*
* Inputs: clk => Clock signal
*         reset => Asynchronous reset
*         stochastic_in => Input stochastic bitstream
*         enable => Enable signal for conversion
*
* Outputs: binary_out => Output binary value (unsigned)
*          conversion_done => Indicates when the STOCHASTIC_BITS length bitstream has been processed
* 
***********/

module stochastic_to_binary_converter #(
    parameter STOCHASTIC_BITS = 256,
    parameter BINARY_WIDTH = 16
)(
    input clk,
    input reset,
    input [STOCHASTIC_BITS-1:0] stochastic_in,
    input enable,
    output reg [BINARY_WIDTH-1:0] binary_out,
    output reg conversion_done
);

    reg [($clog2(STOCHASTIC_BITS+1))-1:0] one_count; // Counter for '1's
    reg [($clog2(STOCHASTIC_BITS))-1:0] bit_index; // Index to iterate through bitstream

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            one_count <= 0;
            bit_index <= 0;
            binary_out <= 0;
            conversion_done <= 0;
        end else if (enable) begin
            if (bit_index < STOCHASTIC_BITS) begin
                if (stochastic_in[bit_index] == 1'b1) begin
                    one_count <= one_count + 1;
                end
                bit_index <= bit_index + 1;
                conversion_done <= 0;
            end else begin
                // Scale the count of '1's to the desired binary output range
                // This is a simple linear scaling.
                // binary_out = (one_count * (2^BINARY_WIDTH - 1)) / STOCHASTIC_BITS
                // For integer arithmetic, this can be approximated or done with more complex logic.
                // Here, a direct multiplication and division (or shift) is used.
                binary_out <= (one_count * ((1 << BINARY_WIDTH) - 1)) / STOCHASTIC_BITS;
                conversion_done <= 1;
                bit_index <= 0; // Reset for next conversion
                one_count <= 0;
            end
        end else begin
            conversion_done <= 0; // Not enabled or conversion not active
        end
    end

endmodule