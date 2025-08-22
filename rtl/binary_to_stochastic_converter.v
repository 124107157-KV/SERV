// FileName: binary_to_stochastic_converter.v
`timescale 1ns / 1ps

/**********
* Binary-to-Stochastic Converter (BSC) - Unipolar Encoding.
* Converts a binary input value to a unipolar stochastic bitstream.
* The probability of a '1' in the output bitstream is proportional to the binary input.
*
* Parameters: BINARY_WIDTH => Bit width of the binary input
*             STOCHASTIC_BITS => Length of the output stochastic bitstream
*             LFSR_WIDTH => Width of the LFSR for random number generation (should be >= BINARY_WIDTH)
*
* Inputs: clk => Clock signal
*         reset => Asynchronous reset
*         binary_in => Binary input value (unsigned)
*         enable => Enable signal for conversion
*
* Outputs: stochastic_out => Output stochastic bitstream
*          conversion_done => Indicates when the STOCHASTIC_BITS length bitstream is generated
* 
***********/

module binary_to_stochastic_converter #(
    parameter BINARY_WIDTH = 16,
    parameter STOCHASTIC_BITS = 256, // Typical bitstream length
    parameter LFSR_WIDTH = 16        // Should be >= BINARY_WIDTH
)(
    input clk,
    input reset,
    input [BINARY_WIDTH-1:0] binary_in,
    input enable,
    output reg [STOCHASTIC_BITS-1:0] stochastic_out,
    output reg conversion_done
);

    reg [LFSR_WIDTH-1:0] random_val;
    wire random_bit; // Not directly used here, but from LFSR
    reg [($clog2(STOCHASTIC_BITS))-1:0] bit_count;
    reg [STOCHASTIC_BITS-1:0] current_stochastic_stream;

    // Instantiate LFSR for random number generation
    lfsr_rng #(.LFSR_WIDTH(LFSR_WIDTH)) lfsr_inst (
        .clk(clk),
        .reset(reset),
        .random_out(random_bit),
        .random_value(random_val)
    );

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            bit_count <= 0;
            current_stochastic_stream <= 0;
            conversion_done <= 0;
        end else if (enable) begin
            if (bit_count < STOCHASTIC_BITS) begin
                // Compare binary_in with random_val to generate stochastic bit
                // Scale binary_in to match LFSR_WIDTH if necessary (assuming BINARY_WIDTH <= LFSR_WIDTH)
                current_stochastic_stream[bit_count] <= (binary_in >= random_val[BINARY_WIDTH-1:0]) ? 1'b1 : 1'b0;
                bit_count <= bit_count + 1;
                conversion_done <= 0;
            end else begin
                conversion_done <= 1;
                bit_count <= 0; // Reset for next conversion
            end
        end else begin
            conversion_done <= 0; // Not enabled or conversion not active
        end
    end

    assign stochastic_out = current_stochastic_stream;

endmodule