// Minimal CNN Peripheral Accelerator for SERV RISC-V System
// Verilog-2001 compliant
// For integration in serv_rf_top as a memory-mapped peripheral accelerator

`timescale 1ns/1ps

module serv_cnn_accel #(
    parameter IMG_W = 8,      // Image width
    parameter IMG_H = 8,      // Image height
    parameter K = 3,          // Kernel size (square)
    parameter OUT_C = 4,      // Output channels (feature maps)
    parameter FC_OUT = 2      // Output classes (e.g., anomaly/normal)
)(
    input wire clk,
    input wire rst,
    input wire start,           // Start CNN computation
    output reg done,            // High when result valid
    input wire [7:0] data_in,   // Data in (pixel/weight) (not used here, see note)
    output reg [3:0] data_out,  // Classification output (argmax result)
    output reg busy             // High while accelerator is processing
);

    // FSM State Encoding
    localparam STATE_IDLE   = 3'd0;
    localparam STATE_LOAD   = 3'd1;
    localparam STATE_CONV   = 3'd2;
    localparam STATE_RELU   = 3'd3;
    localparam STATE_POOL   = 3'd4;
    localparam STATE_FC     = 3'd5;
    localparam STATE_OUTPUT = 3'd6;

    reg [2:0] state, next_state;

    // Internal memories (connected to RAM in real system; here preloaded)
    reg [7:0] img_mem [0:IMG_W*IMG_H-1];          // Input image buffer
    reg [7:0] kernel [0:K*K-1][0:OUT_C-1];        // CNN kernel weights
    reg [7:0] conv_out [0:OUT_C-1][0:IMG_W-K][0:IMG_H-K];
    reg [7:0] relu_out [0:OUT_C-1][0:IMG_W-K][0:IMG_H-K];
    reg [7:0] pool_out [0:OUT_C-1][0:((IMG_W-K+1)/2)-1][0:((IMG_H-K+1)/2)-1];
    reg [7:0] fc_weights [0:FC_OUT-1][0:OUT_C*((IMG_W-K+1)/2)*((IMG_H-K+1)/2)-1];
    reg [15:0] fc_acc [0:FC_OUT-1];

    integer i, j, c, x, y;

    // FSM State register
    always @(posedge clk or posedge rst) begin
        if (rst)
            state <= STATE_IDLE;
        else
            state <= next_state;
    end

    // FSM Next-state logic
    always @(*) begin
        case (state)
            STATE_IDLE:   next_state = (start ? STATE_LOAD : STATE_IDLE);
            STATE_LOAD:   next_state = STATE_CONV;
            STATE_CONV:   next_state = STATE_RELU;
            STATE_RELU:   next_state = STATE_POOL;
            STATE_POOL:   next_state = STATE_FC;
            STATE_FC:     next_state = STATE_OUTPUT;
            STATE_OUTPUT: next_state = STATE_IDLE;
            default:      next_state = STATE_IDLE;
        endcase
    end

    // FSM Output & Layer Operations
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            done <= 0;
            busy <= 0;
            data_out <= 0;
        end else begin
            case (state)
                STATE_IDLE: begin
                    done <= 0;
                    busy <= 0;
                end
                STATE_LOAD: begin
                    // Preload for test/demo (replace with CPU/RAM loads for real HW)
                    for (i = 0; i < IMG_W*IMG_H; i = i + 1)
                        img_mem[i] <= i;
                    for (c = 0; c < OUT_C; c = c + 1)
                        for (i = 0; i < K*K; i = i + 1)
                            kernel[i][c] <= 8'h01 + c;
                    for (i = 0; i < FC_OUT; i = i + 1)
                        for (j = 0; j < OUT_C*((IMG_W-K+1)/2)*((IMG_H-K+1)/2); j = j + 1)
                            fc_weights[i][j] <= 8'h01 + i + j;
                    busy <= 1;
                end
                STATE_CONV: begin
                    for (c = 0; c < OUT_C; c = c + 1) begin
                        for (x = 0; x < IMG_W-K+1; x = x + 1) begin
                            for (y = 0; y < IMG_H-K+1; y = y + 1) begin
                                conv_out[c][x][y] <= 0;
                                for (i = 0; i < K; i = i + 1)
                                    for (j = 0; j < K; j = j + 1)
                                        conv_out[c][x][y] <= conv_out[c][x][y] + img_mem[(x+i)*IMG_W + (y+j)] * kernel[i*K+j][c];
                            end
                        end
                    end
                end
                STATE_RELU: begin
                    for (c = 0; c < OUT_C; c = c + 1)
                        for (x = 0; x < IMG_W-K+1; x = x + 1)
                            for (y = 0; y < IMG_H-K+1; y = y + 1)
                                relu_out[c][x][y] <= (conv_out[c][x][y][7]) ? 8'd0 : conv_out[c][x][y];
                end
                STATE_POOL: begin
                    for (c = 0; c < OUT_C; c = c + 1)
                        for (x = 0; x < (IMG_W-K+1)/2; x = x + 1)
                            for (y = 0; y < (IMG_H-K+1)/2; y = y + 1)
                                pool_out[c][x][y] <= relu_out[c][2*x][2*y]; // Simple pooling (not max)
                end
                STATE_FC: begin
                    for (i = 0; i < FC_OUT; i = i + 1) begin
                        fc_acc[i] <= 0;
                        for (j = 0; j < OUT_C*((IMG_W-K+1)/2)*((IMG_H-K+1)/2); j = j + 1)
                            fc_acc[i] <= fc_acc[i] +
                                pool_out[j/(((IMG_W-K+1)/2)*((IMG_H-K+1)/2))]
                                        [(j/((IMG_H-K+1)/2))%((IMG_W-K+1)/2)]
                                        [j%((IMG_H-K+1)/2)] * fc_weights[i][j];
                    end
                end
                STATE_OUTPUT: begin
                    busy <= 0;
                    if (fc_acc[0] > fc_acc[1])
                        data_out <= 4'd0;
                    else
                        data_out <= 4'd1;
                    done <= 1;
                end
            endcase
        end
    end

endmodule
