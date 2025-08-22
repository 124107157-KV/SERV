// FileName: neural_network_top.v
`timescale 1ns / 1ps

/**********
* Top-level Neural Network implementation for handwritten digit recognition.
* This network uses avg_pooling and two configurable neural layers.
*
* Inputs: clk => clock signal
*         enable => Global NN enable signal
*         reset => Global NN reset signal
*         img => 28x28 pixel input image (784 elements)
*
* Outputs: digit_out => Recognized handwritten digit (0-9)
*          NN_done => Signal indicating NN inference is complete
* 
***********/

module neural_network_top(
    input clk,
    input enable,
    input reset,
    input [7:0] img [0:783],
    output reg [7:0] digit_out,
    output reg NN_done
);

    // --- Average Pooling Layer ---
    reg pool_enable;
    wire finished_pool;
    reg signed [15:0] pooled_img [0:195]; // 14x14 = 196 pixels

    // Pixel value registers for avg_pooling
    wire signed [7:0] pool_in1, pool_in2, pool_in3, pool_in4;
    wire signed [7:0] pool_final;

    // Pixel address registers for avg_pooling
    reg [15:0] pool_in1_addr;
    reg [15:0] pool_in2_addr;
    reg [15:0] pool_in3_addr;
    reg [15:0] pool_in4_addr;
    reg [15:0] pool_final_addr;
    reg [15:0] pool_addr;
    reg [15:0] pool_row;

    // Initial values for avg_pooling addresses
    initial begin
        pool_in1_addr = 16'b0000_0000_0000_0000;
        pool_in2_addr = 16'b0000_0000_0000_0001;
        pool_in3_addr = 16'b0000_0000_0001_1100; // 28 decimal
        pool_in4_addr = 16'b0000_0000_0001_1101; // 29 decimal
        pool_enable = 1'b1;
    end

    avg_pooling AvgPooling_inst(
        .clk(clk),
        .pool_en(pool_enable),
        .in1(pool_in1),
        .in2(pool_in2),
        .in3(pool_in3),
        .in4(pool_in4),
        .out(pool_final),
        .pool_done(finished_pool)
    );

    // Load pixel values from img array
    assign pool_in1 = img[pool_in1_addr];
    assign pool_in2 = img[pool_in2_addr];
    assign pool_in3 = img[pool_in3_addr];
    assign pool_in4 = img[pool_in4_addr];

    always @(posedge clk) begin
        if(reset) begin
            pool_in1_addr <= 0;
            pool_in2_addr <= 1;
            pool_in3_addr <= 28;
            pool_in4_addr <= 29;
            pool_final_addr <= 0;
            pool_row <= 0;
            pool_addr <= 0;
            pool_enable <= 1'b1;
        end
        else if(enable) begin
            if(finished_pool) begin // Average done for current 2x2 block
                pooled_img[pool_final_addr] = pool_final;
                pool_addr = pool_addr + 2; // Move to next 2x2 block horizontally
                pool_row = pool_row + 2;
                if(pool_row == 28) begin // End of current row in original image
                    pool_addr = pool_addr + 28; // Move down two rows in original image
                    pool_row = 0; // Reset row counter
                end
                if(pool_in4_addr == 783) begin // Global averaging done for entire image
                    pool_enable <= 0; // Disable pooling
                end
                else if(pool_in4_addr != 783) begin // Update addresses for next 2x2 block
                    pool_in1_addr <= pool_addr;
                    pool_in2_addr <= pool_addr + 1;
                    pool_in3_addr <= pool_addr + 28;
                    pool_in4_addr <= pool_addr + 29;
                    pool_final_addr <= pool_final_addr + 1;
                end
            end
        end       
    end

    // --- Hidden Layer (Dense Layer 1) ---
    reg dense1_enable;
    wire finished_dense1;
    wire signed [15:0] dense1_output [0:31]; // Output of hidden layer (32 neurons)

    // Biases and weights for Dense Layer 1 (from NN_Handwriting.txt)
    localparam signed [7:0] B_ARRAY_L2 [0:31] = '{ 21, -2, -5, 6, 12, -16, 6, 1, 17, 8, 3, 5, -23, 17, 8, 5, 5, 22, 8, 8, 1, 6, -9, 9, 15, 20, -13, -5, 2, 7, 12, 24 };
    localparam signed [7:0] W_ARRAY_L2 [0:31] [0:195] = '{
        { 7, -1, 9, 12, 17, 36, 29, 35, 33, 44, 25, 23, -8, 7, 0, 11, -10, -4, -6, 5, 5, 28, 27, 29, 12, 28, 1, 9, -1, -11, -41, -26, -8, -22, -22, 6, -1, 12, 16, 14, -11, 12, -8, -31, -31, -2, -21, -7, 11, 5, 1, -2, -26, 1, 4, -14, -5, -4, -44, -20, -2, 6, 20, 10, -3, -31, -26, -28, -5, -18, -3, -15, -44, 1, -1, -12, 1, 14, -16, -17, -1, 2, 28, -28, 7, 2, -16, 9, -7, -8, -1, -5, -4, 7, 10, 12, 25, -41, 10, 4, 4, -2, 0, 8, 6, -12, -26, -7, 3, 7, 31, -18, -11, 9, 6, -19, 9, 29, 6, -9, -17, -18, -1, -3, 24, -25, 15, -7, -15, -12, 14, 12, -5, -6, 5, -7, 4, 5, 23, -25, 3, 0, 6, -1, 21, 32, 13, 14, 17, 10, 5, 13, -8, 11, 2, -1, 10, 18, 10, 16, 14, 14, 11, 1, -20, -23, -24, 20, 0, -28, -2, -13, -9, 1, -16, -29, -32, -17, -37, -43, -6, 18, -1, -1, -29, -39, -56, -33, -52, -46, -56, -41, -33, -16, -5, 1 },
        { -6, -9, 2, -6, 2, -15, -22, 8, -20, 23, 25, 17, -9, 4, -2, 9, 18, 17, -18, -6, -23, -8, 8, 27, 33, -1, -11, -9, -5, 12, 21, 1, -12, -4, 3, 14, 14, 6, -4, -5, -10, 1, 25, 9, 23, -3, -6, -17, 5, 24, 12, -10, -18, -5, 9, 27, -17, 1, 5, -8, -16, -45, -9, 30, -7, -23, -18, -33, 6, 18, -11, -20, -36, -29, -26, -2, 45, 32, -4, -15, -37, -56, -27, 4, -20, -28, -22, -10, 27, 33, 29, -29, -14, -19, -6, -5, -62, -29, -8, -15, -16, 1, -11, -3, -14, -20, 2, 10, 22, 27, -20, -50, 11, -3, 13, 7, -1, 1, 2, -8, 2, 3, 11, 31, -19, -55, -16, 16, 8, 13, 5, 7, 17, 13, 6, 6, -9, 8, -6, -28, 4, -6, -10, 0, 0, -8, -1, 7, 3, 10, -16, -4, 9, -15, 20, 1, -15, -16, -11, 12, 3, -7, -1, 0, -9, -5, -12, 27, 5, 8, 16, 33, 33, 3, -6, -2, -3, 19, 12, 11, 22, 20, 4, -4, 7, -12, -34, -42, -56, -12, 17, 0, 0, -13, 8, -7 },
        // ... (Remaining weights are presumed to be declared correctly)
        { -8, 5, 8, 21, 11, 9, -1, 6, 3, 42, 37, 16, -2, 9, 3, -3, 0, 27, -1, 6, 12, 27, 12, 12, 25, 21, 4, 3, 6, -15, 0, 3, 11, -10, -9, -7, -13, -14, -10, -5, -30, -35, -25, 12, 0, 17, 2, -13, 1, -1, -1, -17, -17, -14, -35, -3, 32, 9, 13, 8, -2, 1, 1, 2, -15, -27, -27, -3, -13, -8, 23, 15, -5, -3, -7, 11, 19, 16, 11, -2, -15, -12, -29, -26, 24, 13, -4, 3, 3, 30, 2, 16, 35, 16, 2, -10, -26, -17, 11, 16, -8, 0, 4, 29, 0, 5, 32, 27, 14, 22, -35, -15, -9, -15, 5, 1, -2, 7, -21, 17, 32, 14, 15, -2, 0, 4, 10, -4, -5, -28, -27, -26, -10, 16, -12, -18, -29, -31, 8, 23, -14, -18, -19, -33, -28, -10, 4, -9, -21, -38, -57, -42, -11, -7, 1, -49, -44, -11, 6, 1, -13, -20, -8, -20, -17, -35, -16, -12, -7, -17, -25, -2, 4, 3, -10, 3, 8, 11, 17, 3, -11, -3, 9, 7, -8, 22, 39, 14, 9, 31, 10, 22, 6, 11, 5, 9 }
    };

    neural_layer_dual_alu # (
        .NEURON_NB(32),
        .IN_SIZE(196),
        .DATA_WIDTH(8),
        .USE_STOCHASTIC_ACTIVATION(0) // Use binary ReLU for hidden layer
    ) hidden_layer_inst (
        .clk(clk),
        .enable(dense1_enable),
        .reset(reset),
        .in_data(pooled_img),
        .weights(W_ARRAY_L2),
        .biases(B_ARRAY_L2),
        .layer_out(dense1_output),
        .layer_done(finished_dense1)
    );

    always @(posedge clk) begin
        if(reset) begin
            dense1_enable <= 0;
        end
        else if(enable) begin
            // Enable hidden layer when pooling is done and hidden layer is not yet done
            if(pool_enable == 0 && finished_dense1 == 0) begin 
                dense1_enable <= 1;
            end
            else dense1_enable <= 0; // Disable once done or if pooling not finished
        end
    end

    // --- Output Layer (Dense Layer 2) ---
    reg dense2_enable;
    wire finished_dense2;
    wire signed [15:0] dense2_output [0:9]; // Output of output layer (10 neurons)

    // Biases and weights for Dense Layer 2 (from NN_Handwriting.txt)
    localparam signed [7:0] B_ARRAY_L3 [0:9] = '{ -11, 10, 14, -23, -1, 8, 0, 13, 1, -11 };
    localparam signed [7:0] W_ARRAY_L3 [0:9] [0:31] = '{
        { 33, -57, -6, -23, -11, -48, -53, 46, -54, -24, 4, 9, -2, -40, -29, 20, -6, 45, -85, 2, -57, 45, 40, -21, -23, -2, -105, -19, -40, 28, -7, 15 },
        { 30, 34, 29, -19, -12, -47, 50, -69, 64, -25, 38, 35, -43, 19, -17, -61, 62, -31, -30, -69, 43, -67, -60, 5, 46, -1, -50, -64, 39, -39, -28, -28 },
        { 2, -48, 30, 33, 69, -11, 28, -1, 28, 25, -22, 58, -11, -120, -6, -28, -72, -12, -33, -4, 1, 81, 21, -22, 24, -100, -59, -59, -20, 27, 37, 39 },
        { -37, 61, -6, -19, -5, 60, 14, 8, 28, -29, -106, 53, 32, -18, -27, -16, -1, -45, 43, 40, 18, -28, 25, -37, -7, -23, 52, -45, 16, -28, 6, -53 },
        { -9, 57, 20, 19, -96, -25, -47, 19, 43, -19, 8, -68, -37, 37, 28, -8, 13, -15, 17, -40, -33, 2, -100, 35, 5, -52, 12, 34, -75, -92, 30, 17 },
        { 16, -8, -127, -69, 28, -82, 29, -47, -7, -19, -11, -55, -10, 37, -46, -14, 26, -1, 39, 35, -37, -5, 10, -20, -35, 84, 55, 13, 37, 25, -5, -36 },
        { 40, 29, -74, -1, -16, 71, -91, -13, -92, 24, 32, -15, -5, -12, -73, -70, 0, -25, -85, 38, -63, 2, -53, 36, 39, 4, -93, 24, 26, 19, 12, 37 },
        { -40, -42, -10, 45, 26, 8, 25, 8, -33, -40, -32, 34, -40, -38, 32, 19, 42, -20, -18, 41, 21, -41, 24, 7, -25, 37, -93, -32, -56, -52, -48, 55 },
        { 4, -73, 17, -34, 17, 0, -34, -9, -50, 10, 29, -75, 37, 10, -19, 0, -63, -1, 12, 2, 9, -63, 7, 40, -12, -54, -17, 26, -10, -5, -7, -89 },
        { -82, -31, 23, -61, -34, 18, -52, -24, -1, 12, 29, -74, -11, 14, 34, 33, 23, -13, -21, -98, -48, 32, 26, -35, 7, 3, 68, 14, -25, -11, -86, 27 }
    };

    neural_layer_dual_alu # (
        .NEURON_NB(10),
        .IN_SIZE(32),
        .DATA_WIDTH(8),
        .USE_STOCHASTIC_ACTIVATION(0) // Use binary ReLU for output layer
    ) output_layer_inst (
        .clk(clk),
        .enable(dense2_enable),
        .reset(reset),
        .in_data(dense1_output),
        .weights(W_ARRAY_L3),
        .biases(B_ARRAY_L3),
        .layer_out(dense2_output),
        .layer_done(finished_dense2)
    );

    always @(posedge clk) begin
        if(reset) begin
            dense2_enable <= 0;
        end
        else if(enable) begin
            // Enable output layer when hidden layer is done and output layer is not yet done
            if(pool_enable == 0 && finished_dense1 == 1 && finished_dense2 == 0) begin 
                dense2_enable <= 1;
            end
            else dense2_enable <= 0; // Disable once done or if previous layers not finished
        end
    end

    // --- Select Max Layer ---
    reg max_enable;
    wire digit_recog_done;
    reg [7:0] digit_reg; // Internal register for the recognized digit

    select_max SelectMax_inst(
        .clk(clk),
        .enable(max_enable),
        .reset(reset),
        .in_data(dense2_output),
        .digit(digit_reg),
        .layer_done(digit_recog_done)
    );

    always @(posedge clk) begin
        if(reset) begin
            max_enable <= 0;
        end
        else if(enable) begin
            // Enable select max when output layer is done
            if(finished_dense2 == 1) begin 
                max_enable <= 1;
            end
            else max_enable <= 0;
        end
    end

    // Final outputs
    always @(posedge clk) begin
        digit_out <= digit_reg;
        NN_done <= digit_recog_done; // NN is done when select max is done
    end

endmodule