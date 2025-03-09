`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 24.02.2025 02:24:41
// Design Name: 
// Module Name: parallel_serial_if
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


`timescale 1ns / 1ps
`default_nettype none

module parallel_serial_if #(parameter DATA_WIDTH = 171)(
    input  wire                   clk,
    input  wire                   rst,
    input  wire [DATA_WIDTH-1:0]  parallel_in,   // Data to be transmitted serially.
    output reg  [DATA_WIDTH-1:0]  parallel_out,  // Data received from serial line.
    output reg                    ser_clk,       // Serial clock output.
    output reg                    ser_data_out,  // Serial data out.
    input  wire                   ser_data_in,   // Serial data in.
    output reg                    ser_cs         // Chip select (active low).
);
    // We'll implement a simple state machine that continuously transmits DATA_WIDTH bits.
    reg [$clog2(DATA_WIDTH):0] bit_cnt;
    reg transmitting;

    always @(posedge clk or posedge rst) begin
        if (rst) begin
            bit_cnt       <= 0;
            ser_cs        <= 1'b1;
            ser_clk       <= 1'b0;
            ser_data_out  <= 1'b0;
            transmitting  <= 1'b0;
            parallel_out  <= {DATA_WIDTH{1'b0}};
        end else begin
            // Start transmission if not already transmitting.
            if (!transmitting) begin
                transmitting <= 1'b1;
                ser_cs       <= 1'b0; // Assert chip select.
                bit_cnt      <= 0;
            end else begin
                // Toggle serial clock.
                ser_clk <= ~ser_clk;
                // On falling edge of ser_clk, output the next bit and sample the input.
                if (ser_clk == 1'b0) begin
                    ser_data_out <= parallel_in[DATA_WIDTH-1 - bit_cnt];
                    parallel_out[DATA_WIDTH-1 - bit_cnt] <= ser_data_in;
                    bit_cnt <= bit_cnt + 1;
                    if (bit_cnt == DATA_WIDTH-1) begin
                        transmitting <= 1'b0;
                        ser_cs <= 1'b1; // End transmission.
                    end
                end
            end
        end
    end

endmodule

`default_nettype wire

