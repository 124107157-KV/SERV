`default_nettype none
`timescale 1ns/1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.03.2025 04:03:27
// Design Name: 
// Module Name: axi_stream_to_parallel
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


//==============================================================================
// AXI4-Stream to Parallel Conversion Module (for Write Direction)
// Packs 6 32-bit words into a 171-bit wide bus.
//==============================================================================
module axi_stream_to_parallel #(
    parameter DATA_WIDTH_PAR = 171,
    parameter WORDS = 6,              // 6*32 = 192 bits; using upper 171 bits
    parameter DATA_WIDTH_STR = 32
)(
    input  wire                   clk,
    input  wire                   rst,
    // AXI4-Stream Slave Interface (Write)
    input  wire [DATA_WIDTH_STR-1:0] s_tdata,
    input  wire                   s_tvalid,
    output reg                    s_tready,
    input  wire                   s_tlast,
    // Parallel output bus to SERV core
    output reg [DATA_WIDTH_PAR-1:0] parallel_out,
    output reg                    valid_out
);
    // Internal register to collect words
    reg [WORDS*DATA_WIDTH_STR-1:0] word_buf;
    reg [$clog2(WORDS):0] word_cnt;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            word_buf   <= 0;
            word_cnt   <= 0;
            s_tready   <= 1'b1;
            valid_out  <= 1'b0;
            parallel_out <= 0;
        end else begin
            // When tvalid is asserted, latch the word
            if (s_tvalid && s_tready) begin
                // Shift in new data word into the buffer (MSB first)
                word_buf <= { word_buf[(WORDS-1)*DATA_WIDTH_STR-1:0], s_tdata };
                word_cnt <= word_cnt + 1;
                if (word_cnt == WORDS-1) begin
                    // We've collected all words.
                    // We extract the upper 171 bits from the 192-bit buffer.
                    parallel_out <= word_buf[WORDS*DATA_WIDTH_STR-1 -: DATA_WIDTH_PAR];
                    valid_out <= 1'b1;
                    word_cnt <= 0;
                end
            end
            // Accept new words if not full
            s_tready <= (word_cnt < WORDS);
            // Clear valid_out after one cycle
            if(valid_out)
                valid_out <= 1'b0;
        end
    end
endmodule
