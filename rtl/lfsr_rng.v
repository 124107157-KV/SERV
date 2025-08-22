`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10.07.2025 02:47:29
// Design Name: 
// Module Name: lfsr_rng
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


// FileName: lfsr_rng.v
`timescale 1ns / 1ps

/**********
* Simple LFSR-based Pseudo-Random Number Generator.
* Used by BSC and SBC for stochastic operations.
*
* Parameters: LFSR_WIDTH => Width of the LFSR (determines period and randomness)
*
* Inputs: clk => Clock signal
*         reset => Asynchronous reset
*
* Outputs: random_out => LFSR output bit
*          random_value => LFSR current state (for comparison in BSC)
* 
***********/

module lfsr_rng #(parameter LFSR_WIDTH = 16)(
    input clk,
    input reset,
    output reg random_out,
    output reg [LFSR_WIDTH-1:0] random_value
);

    // Initial seed (must not be all zeros for a maximal LFSR)
    localparam [LFSR_WIDTH-1:0] SEED = {{(LFSR_WIDTH-1){1'b0}}, 1'b1};

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            random_value <= SEED;
            random_out <= SEED[0];
        end else begin
            // Example for a 16-bit LFSR (X^16 + X^14 + X^13 + X^11 + 1)
            // Tap positions depend on LFSR_WIDTH for maximal length
            case (LFSR_WIDTH)
                8: random_value <= {random_value[6:0], random_value[7] ^ random_value[5] ^ random_value[4] ^ random_value[3]};
                16: random_value <= {random_value[14:0], random_value[15] ^ random_value[13] ^ random_value[12] ^ random_value[10]};
                32: random_value <= {random_value[30:0], random_value[31] ^ random_value[21] ^ random_value[1] ^ random_value[0]};
                default: random_value <= {random_value[LFSR_WIDTH-2:0], random_value[LFSR_WIDTH-1] ^ random_value[LFSR_WIDTH-2]}; // Generic, not necessarily maximal
            endcase
            random_out <= random_value[0]; // Output the LSB
        end
    end

endmodule
