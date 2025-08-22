`timescale 1ns / 1ps

module serv_stoc_alu #(
    parameter W = 1,
    parameter B = W - 1
)(
    input wire clk,
    input wire i_en,
    input wire rst,  // Added reset for proper initialization
    input wire [4:0] i_alu_ctrl_bus,
    input wire i_matrix_op,
    input wire [2:0] i_alu_cmp_ctrl_bus,
    input wire [(4*W) - 1:0] i_alu_data_bus,
    output reg [B:0] o_rd,
    output wire o_cmp
);

    // Extract control and data signals
    wire [2:0] i_rd_sel = i_alu_ctrl_bus[4:2];
    wire [1:0] i_bool_op = i_alu_ctrl_bus[1:0];
    wire i_sub = i_alu_cmp_ctrl_bus[2];
    wire i_cmp_eq = i_alu_cmp_ctrl_bus[1];
    wire i_cmp_sig = i_alu_cmp_ctrl_bus[0];
    
    // For W=1, extract 1-bit signals properly
    wire i_rs1 = i_alu_data_bus[3];      // Bit 3 (MSB of 4-bit bus for W=1)
    wire i_op_b = i_alu_data_bus[2];     // Bit 2
    wire i_buf = i_alu_data_bus[1];      // Bit 1
    wire i_mat_element = i_alu_data_bus[0]; // Bit 0 (LSB)

    // === BASIC STOCHASTIC OPERATIONS (1-bit) ===
    wire result_stoc_add = i_rs1 | i_op_b;        // Stochastic Addition (OR)
    wire result_stoc_sub = i_rs1 ^ i_op_b;        // Stochastic Subtraction (XOR)
    wire result_stoc_mult = i_rs1 & i_op_b;       // Stochastic Multiplication (AND)
    wire result_stoc_div = i_rs1 & (~i_op_b);     // Stochastic Division

    // === ADVANCED MATHEMATICAL FUNCTIONS (1-bit) ===
    wire result_exp = i_rs1;                      // For 1-bit: X^2 = X (idempotent)
    wire result_log = i_rs1;                      // For 1-bit: log(X) ≈ X
    wire result_sqrt = i_rs1;                     // For 1-bit: sqrt(X) ≈ X
    wire result_poly = (i_rs1 & i_op_b) | i_rs1; // Polynomial approximation
    wire result_reciprocal = (~i_rs1);            // Reciprocal (1-X)
    wire result_power = i_rs1 & i_op_b;           // Power function X^Y

    // === LFSR for randomness ===
    reg [7:0] lfsr;

    // === ACTIVATION FUNCTIONS (1-bit stochastic) ===
    // Sigmoid: Use probabilistic threshold with LFSR
    wire result_sigmoid_stoc = i_rs1 | (lfsr[0] & lfsr[1]); // Slightly boost probability
    
    // Tanh: Bipolar-like behavior with randomness
    wire result_tanh_stoc = i_rs1 ^ (lfsr[2] & lfsr[3]);
    
    // ReLU: For 1-bit stochastic, just pass through (probabilities are always positive)
    wire result_relu_stoc = i_rs1;
    
    // Leaky ReLU: Small leak for "negative" values
    wire result_leaky_relu_stoc = i_rs1 | (lfsr[4] & (~i_rs1));

    // Advanced activation functions
    wire result_swish_stoc = i_rs1 & result_sigmoid_stoc;    // x * sigmoid(x)
    wire result_gelu_stoc = i_rs1 & result_tanh_stoc;       // GELU approximation

    // Softmax (for single neuron in 1-bit)
    wire result_softmax_basic = i_rs1;
    wire result_softmax_enhanced = i_rs1 ^ lfsr[5]; // Add randomness

    // === NEURAL NETWORK SPECIFIC FUNCTIONS (1-bit) ===
    wire result_dot = result_stoc_mult | i_buf;              // Dot product accumulation
    wire result_conv_basic = result_stoc_mult | result_dot;  // Basic convolution
    wire result_conv_enhanced = (i_rs1 & i_op_b) | (i_buf & i_mat_element);

    // Pooling operations (1-bit)
    wire result_pool_max = i_rs1 | i_op_b;                  // Max pooling (OR)
    wire result_pool_avg = i_rs1 ^ i_op_b;                  // Average pooling approximation

    // Batch Normalization (1-bit)
    wire result_batch_norm = (i_rs1 ^ i_op_b) & i_buf;

    // Attention Mechanism (1-bit)
    wire result_attention = (i_rs1 & i_op_b) | (i_buf & i_mat_element);

    // === LOGICAL OPERATIONS (1-bit) ===
    wire result_and = i_rs1 & i_op_b;
    wire result_or = i_rs1 | i_op_b;
    wire result_xor = i_rs1 ^ i_op_b;
    wire result_xnor = ~(i_rs1 ^ i_op_b);
    wire result_not = ~i_rs1;
    wire result_nor = ~(i_rs1 | i_op_b);
    wire result_nand = ~(i_rs1 & i_op_b);
    wire result_parity = i_rs1;  // For 1-bit, parity is the bit itself
    wire result_majority = (i_rs1 & i_op_b) | (i_rs1 & i_mat_element) | (i_op_b & i_mat_element);

    // Boolean logic based on i_bool_op
    wire result_logic = (i_bool_op == 2'b00) ? result_and :
                       (i_bool_op == 2'b01) ? result_or :
                       (i_bool_op == 2'b10) ? result_xor :
                       result_xnor;

    // === MATRIX OPERATIONS (1-bit) ===
    wire result_mat_add = i_rs1 | i_mat_element;
    wire result_mat_sub = i_rs1 ^ i_mat_element;
    wire result_mat_mult = i_rs1 & i_mat_element;
    wire result_scalar_mult = i_rs1 & i_op_b;
    wire result_mat_transpose = i_mat_element;

    // === RANDOMNESS AND NOISE (1-bit) ===
    wire result_dropout = (lfsr[2:0] > 3'd2) ? i_rs1 : 1'b0; // ~25% dropout
    wire result_stochastic_round = i_rs1 ^ lfsr[0];           // Add randomness
    wire result_gaussian = i_rs1 & lfsr[1];                  // Gaussian noise approximation

    // === COMPARISON OPERATIONS (1-bit) ===
    wire result_comparator = i_rs1 & (~i_op_b);  // i_rs1 > i_op_b for 1-bit
    wire result_slt = (~i_rs1) & i_op_b;         // i_rs1 < i_op_b for 1-bit

    // === ADDITIONAL OPERATION ACCESS (1-bit) ===
    wire aux_result;
    assign aux_result = (i_alu_ctrl_bus == 5'b11000) ? result_dropout :
                       (i_alu_ctrl_bus == 5'b11001) ? result_batch_norm :
                       (i_alu_ctrl_bus == 5'b11010) ? result_attention :
                       (i_alu_ctrl_bus == 5'b11011) ? result_stochastic_round :
                       (i_alu_ctrl_bus == 5'b11100) ? result_gaussian :
                       (i_alu_ctrl_bus == 5'b11101) ? result_power :
                       (i_alu_ctrl_bus == 5'b11110) ? result_parity :
                       (i_alu_ctrl_bus == 5'b11111) ? result_majority :
                       1'b0;

    // === LFSR INITIALIZATION AND UPDATE ===
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            lfsr <= 8'hA5; // Initialize LFSR with non-zero seed
        end else begin
            // Update LFSR for randomness every cycle
            lfsr <= {lfsr[6:0], lfsr[7] ^ lfsr[5] ^ lfsr[4] ^ lfsr[3]};
        end
    end

    // === SINGLE CONSOLIDATED OUTPUT SELECTION ===
    // This is the ONLY always block that assigns to o_rd
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            o_rd <= 1'b0;
        end else if (i_en) begin
            // Check for auxiliary operations first (highest priority)
            if (i_alu_ctrl_bus[4:3] == 2'b11) begin
                o_rd <= aux_result;
            end else begin
                // Main case statement for standard operations
                case ({i_matrix_op, i_rd_sel})
                    // === BASIC ARITHMETIC (i_matrix_op=0, i_rd_sel=000-011) ===
                    4'b0000: o_rd <= result_stoc_add;         // Stochastic addition (OR)
                    4'b0001: o_rd <= result_stoc_sub;         // Stochastic subtraction (XOR)
                    4'b0010: o_rd <= result_stoc_mult;        // Stochastic multiplication (AND)
                    4'b0011: o_rd <= result_stoc_div;         // Stochastic division

                    // === ACTIVATION FUNCTIONS (i_matrix_op=0, i_rd_sel=100-111) ===
                    4'b0100: o_rd <= result_sigmoid_stoc;     // Sigmoid
                    4'b0101: o_rd <= result_tanh_stoc;        // Tanh
                    4'b0110: o_rd <= result_relu_stoc;        // ReLU
                    4'b0111: o_rd <= result_leaky_relu_stoc;  // Leaky ReLU

                    // === ADVANCED FUNCTIONS (i_matrix_op=1, i_rd_sel=000-111) ===
                    4'b1000: o_rd <= result_exp;              // Exponentiation
                    4'b1001: o_rd <= result_log;              // Logarithm
                    4'b1010: o_rd <= result_sqrt;             // Square root
                    4'b1011: o_rd <= result_poly;             // Polynomial
                    4'b1100: o_rd <= result_swish_stoc;       // Swish activation
                    4'b1101: o_rd <= result_gelu_stoc;        // GELU activation
                    4'b1110: o_rd <= result_softmax_enhanced; // Softmax
                    4'b1111: o_rd <= result_reciprocal;       // Reciprocal

                    default: o_rd <= 1'b0;
                endcase
            end
        end
    end

    // === COMPARATOR OUTPUT ===
    assign o_cmp = result_comparator;

endmodule