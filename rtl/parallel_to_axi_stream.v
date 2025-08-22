`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 05.03.2025 04:04:38
// Design Name: 
// Module Name: parallel_to_axi_stream
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
// Parallel to AXI4-Stream Conversion Module (for Read Direction)
// Unpacks a 171-bit wide internal bus into 6 32-bit words on an AXI4-Stream Master.
//==============================================================================
module parallel_to_axi_stream #(
    parameter DATA_WIDTH_PAR = 171,
    parameter WORDS = 6,              // 6 words (192 bits total, using upper 171 bits)
    parameter DATA_WIDTH_STR = 32
)(
    input  wire                    clk,
    input  wire                    rst,
    // Parallel input from SERV core
    input  wire [DATA_WIDTH_PAR-1:0] parallel_in,
    input  wire                    valid_in,
    // AXI4-Stream Master Interface (Read)
    output reg [DATA_WIDTH_STR-1:0] m_tdata,
    output reg                    m_tvalid,
    input  wire                   m_tready,
    output reg                    m_tlast
);
    // Internal buffer: we pack 6 words into 192 bits
    reg [WORDS*DATA_WIDTH_STR-1:0] word_buf;
    reg [$clog2(WORDS):0] word_cnt;
    reg transmitting;

    always @(posedge clk or posedge rst) begin
        if(rst) begin
            word_buf   <= 0;
            word_cnt   <= 0;
            transmitting <= 1'b0;
            m_tvalid   <= 1'b0;
            m_tlast    <= 1'b0;
            m_tdata    <= 0;
        end else begin
            // When new parallel data is valid, load it into the buffer
            if(valid_in && !transmitting) begin
                // Place the 171 bits into the upper bits of a 192-bit register
                word_buf <= { parallel_in, { (WORDS*DATA_WIDTH_STR - DATA_WIDTH_PAR){1'b0} } };
                transmitting <= 1'b1;
                word_cnt <= 0;
            end

            if(transmitting) begin
                m_tvalid <= 1'b1;
                // Output the word corresponding to the current count
                m_tdata <= word_buf[(WORDS-1-word_cnt)*DATA_WIDTH_STR +: DATA_WIDTH_STR];
                if(m_tready) begin
                    if(word_cnt == WORDS-1) begin
                        m_tlast <= 1'b1;
                        transmitting <= 1'b0;
                        word_cnt <= 0;
                    end else begin
                        word_cnt <= word_cnt + 1;
                        m_tlast <= 1'b0;
                    end
                end
            end else begin
                m_tvalid <= 1'b0;
                m_tlast  <= 1'b0;
            end
        end
    end
endmodule