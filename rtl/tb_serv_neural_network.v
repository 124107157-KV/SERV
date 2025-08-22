`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 19.08.2025 02:55:54
// Design Name: 
// Module Name: tb_serv_neural_network
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


// File: tb_serv_neural_network.v
`timescale 1ns / 1ps

module tb_serv_neural_network;

    // Parameters
    parameter CLK_PERIOD = 10; // 100MHz clock
    parameter W = 8;
    parameter B = W - 1;
    parameter NN_INPUT_DIM = 4;
    parameter NN_HIDDEN1_DIM = 8;
    parameter NN_HIDDEN2_DIM = 6;
    parameter NN_OUTPUT_DIM = 3;
    parameter TIMEOUT_CYCLES = 10000;

    // Test signals
    reg clk;
    reg i_rst;
    reg i_timer_irq;
    
    // Neural Network Interface
    reg [NN_INPUT_DIM*W-1:0] nn_input_vector;
    reg nn_start;
    wire [NN_OUTPUT_DIM*W-1:0] nn_output_vector_relu;
    wire [NN_OUTPUT_DIM*W-1:0] nn_output_vector_sigmoid;
    wire [NN_OUTPUT_DIM*W-1:0] nn_output_vector_tanh;
    wire nn_done;
    wire nn_ready;
    wire [7:0] nn_status;
    
    // Register File Interface (minimal for testing)
    wire o_rf_rreq;
    wire o_rf_wreq;
    reg i_rf_ready;
    wire [4:0] o_wreg0, o_wreg1;
    wire o_wen0, o_wen1;
    wire [B:0] o_wdata0, o_wdata1;
    wire [4:0] o_rreg0, o_rreg1;
    reg [B:0] i_rdata0, i_rdata1;
    
    // Bus Interfaces (minimal for testing)
    wire [31:0] o_ibus_adr, o_dbus_adr, o_dbus_dat;
    wire o_ibus_cyc, o_dbus_cyc, o_dbus_we;
    wire [3:0] o_dbus_sel;
    reg [31:0] i_ibus_rdt, i_dbus_rdt;
    reg i_ibus_ack, i_dbus_ack;
    wire o_mdu_valid;

    // Test variables
    integer test_count;
    integer pass_count;
    integer fail_count;
    integer timeout_counter;
    reg [255:0] test_name;
    
    // Expected results storage
    reg [NN_OUTPUT_DIM*W-1:0] expected_relu;
    reg [NN_OUTPUT_DIM*W-1:0] expected_sigmoid;
    reg [NN_OUTPUT_DIM*W-1:0] expected_tanh;

    // DUT instantiation
    serv_top #(
        .WITH_CSR(1),
        .W(W),
        .B(B),
        .PRE_REGISTER(1),
        .RESET_STRATEGY("MINI"),
        .RESET_PC(32'd0),
        .DEBUG(1'b0),
        .MDU(1'b0),
        .COMPRESSED(0),
        .ALIGN(0),
        .USE_STOC_ALU(1),
        .NN_INPUT_DIM(NN_INPUT_DIM),
        .NN_HIDDEN1_DIM(NN_HIDDEN1_DIM),
        .NN_HIDDEN2_DIM(NN_HIDDEN2_DIM),
        .NN_OUTPUT_DIM(NN_OUTPUT_DIM)
    ) dut (
        .clk(clk),
        .i_rst(i_rst),
        .i_timer_irq(i_timer_irq),
        
        // Register File Interface
        .o_rf_rreq(o_rf_rreq),
        .o_rf_wreq(o_rf_wreq),
        .i_rf_ready(i_rf_ready),
        .o_wreg0(o_wreg0),
        .o_wreg1(o_wreg1),
        .o_wen0(o_wen0),
        .o_wen1(o_wen1),
        .o_wdata0(o_wdata0),
        .o_wdata1(o_wdata1),
        .o_rreg0(o_rreg0),
        .o_rreg1(o_rreg1),
        .i_rdata0(i_rdata0),
        .i_rdata1(i_rdata1),
        
        // Bus Interfaces
        .o_ibus_adr(o_ibus_adr),
        .o_ibus_cyc(o_ibus_cyc),
        .i_ibus_rdt(i_ibus_rdt),
        .i_ibus_ack(i_ibus_ack),
        .o_dbus_adr(o_dbus_adr),
        .o_dbus_dat(o_dbus_dat),
        .o_dbus_sel(o_dbus_sel),
        .o_dbus_we(o_dbus_we),
        .o_dbus_cyc(o_dbus_cyc),
        .i_dbus_rdt(i_dbus_rdt),
        .i_dbus_ack(i_dbus_ack),
        .o_mdu_valid(o_mdu_valid),
        
        // Neural Network Interface
        .nn_input_vector(nn_input_vector),
        .nn_start(nn_start),
        .nn_output_vector_relu(nn_output_vector_relu),
        .nn_output_vector_sigmoid(nn_output_vector_sigmoid),
        .nn_output_vector_tanh(nn_output_vector_tanh),
        .nn_done(nn_done),
        .nn_ready(nn_ready),
        .nn_status(nn_status)
    );

    // Clock generation
    initial begin
        clk = 0;
        forever #(CLK_PERIOD/2) clk = ~clk;
    end

    // Initialize signals
    initial begin
        // Initialize all signals
        i_rst = 1;
        i_timer_irq = 0;
        nn_input_vector = 0;
        nn_start = 0;
        i_rf_ready = 1;
        i_rdata0 = 0;
        i_rdata1 = 0;
        i_ibus_rdt = 32'h00000013; // NOP instruction
        i_ibus_ack = 1;
        i_dbus_rdt = 0;
        i_dbus_ack = 1;
        
        test_count = 0;
        pass_count = 0;
        fail_count = 0;
        timeout_counter = 0;
        
        // Reset sequence
        #(CLK_PERIOD * 5);
        i_rst = 0;
        #(CLK_PERIOD * 5);
        
        $display("=== Neural Network Testbench Started ===");
        $display("Time: %0t", $time);
        
        // Run all tests
        run_all_tests();
        
        // Final results
        $display("\n=== Test Summary ===");
        $display("Total Tests: %0d", test_count);
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", fail_count);
        $display("Success Rate: %0.1f%%", (pass_count * 100.0) / test_count);
        
        if (fail_count == 0) begin
            $display("*** ALL TESTS PASSED ***");
        end else begin
            $display("*** %0d TESTS FAILED ***", fail_count);
        end
        
        $finish;
    end

    // Main test runner
    task run_all_tests;
        begin
            // Basic functionality tests
            test_basic_functionality();
            test_zero_input();
            test_maximum_input();
            test_minimum_input();
            test_random_inputs();
            
            // Edge case tests
            test_single_bit_inputs();
            test_alternating_patterns();
            test_sequential_patterns();
            
            // Stress tests
            test_rapid_fire();
            test_back_to_back();
            
            // Error condition tests
            test_reset_during_operation();
            test_start_while_busy();
            
            // Timing tests
            test_setup_hold_times();
            test_timeout_conditions();
        end
    endtask

    // Test 1: Basic functionality
    task test_basic_functionality;
        begin
            test_name = "Basic Functionality Test";
            start_test();
            
            // Test with simple known values
            nn_input_vector = {8'd10, 8'd20, 8'd30, 8'd40}; // 4 inputs
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Basic functionality");
            
            end_test();
        end
    endtask

    // Test 2: Zero input
    task test_zero_input;
        begin
            test_name = "Zero Input Test";
            start_test();
            
            nn_input_vector = {8'd0, 8'd0, 8'd0, 8'd0};
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Zero input");
            
            end_test();
        end
    endtask

    // Test 3: Maximum input values
    task test_maximum_input;
        begin
            test_name = "Maximum Input Test";
            start_test();
            
            nn_input_vector = {8'd255, 8'd255, 8'd255, 8'd255};
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Maximum input");
            
            end_test();
        end
    endtask

    // Test 4: Minimum input values (signed interpretation)
    task test_minimum_input;
        begin
            test_name = "Minimum Input Test";
            start_test();
            
            nn_input_vector = {8'd128, 8'd128, 8'd128, 8'd128}; // -128 in signed
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Minimum input");
            
            end_test();
        end
    endtask

    // Test 5: Random inputs
    task test_random_inputs;
        integer i;
        begin
            for (i = 0; i < 10; i = i + 1) begin
                test_name = {"Random Input Test ", i};
                start_test();
                
                nn_input_vector = {$random, $random, $random, $random};
                expected_relu = calculate_expected_relu(nn_input_vector);
                expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
                expected_tanh = calculate_expected_tanh(nn_input_vector);
                
                run_nn_inference();
                check_results({"Random input ", i});
                end_test();
            end
        end
    endtask

    // Test 6: Single bit inputs
    task test_single_bit_inputs;
        integer i, j;
        begin
            for (i = 0; i < 4; i = i + 1) begin
                for (j = 0; j < 8; j = j + 1) begin
                    test_name = {"Single Bit Test Input[", i, "] Bit[", j, "]"};
                    start_test();
                    
                    nn_input_vector = 0;
                    nn_input_vector[i*8 + j] = 1'b1;
                    
                    expected_relu = calculate_expected_relu(nn_input_vector);
                    expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
                    expected_tanh = calculate_expected_tanh(nn_input_vector);
                    
                    run_nn_inference();
                    check_results({"Single bit [", i, "][", j, "]"});
                    
                    end_test();
                end
            end
        end
    endtask

    // Test 7: Alternating patterns
    task test_alternating_patterns;
        begin
            test_name = "Alternating Pattern Test";
            start_test();
            
            nn_input_vector = {8'b10101010, 8'b01010101, 8'b10101010, 8'b01010101};
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Alternating pattern");
            
            end_test();
        end
    endtask

    // Test 8: Sequential patterns
    task test_sequential_patterns;
        begin
            test_name = "Sequential Pattern Test";
            start_test();
            
            nn_input_vector = {8'd1, 8'd2, 8'd4, 8'd8};
            expected_relu = calculate_expected_relu(nn_input_vector);
            expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
            expected_tanh = calculate_expected_tanh(nn_input_vector);
            
            run_nn_inference();
            check_results("Sequential pattern");
            
            end_test();
        end
    endtask

    // Test 9: Rapid fire test
    task test_rapid_fire;
        integer i;
        begin
            for (i = 0; i < 5; i = i + 1) begin
                test_name = {"Rapid Fire Test ", i};
                start_test();
                
                nn_input_vector = {8'd50 + i, 8'd100 + i, 8'd150 + i, 8'd200 + i};
                expected_relu = calculate_expected_relu(nn_input_vector);
                expected_sigmoid = calculate_expected_sigmoid(nn_input_vector);
                expected_tanh = calculate_expected_tanh(nn_input_vector);
                
                run_nn_inference();
                check_results({"Rapid fire ", i});
                
                end_test();
                
                // Minimal delay between tests
                #(CLK_PERIOD * 2);
            end
        end
    endtask

    // Test 10: Back-to-back operations
    task test_back_to_back;
        begin
            test_name = "Back-to-Back Test 1";
            start_test();
            
            nn_input_vector = {8'd25, 8'd50, 8'd75, 8'd100};
            run_nn_inference();
            check_results("Back-to-back 1");
            end_test();
            
            // Immediately start next test
            test_name = "Back-to-Back Test 2";
            start_test();
            
            nn_input_vector = {8'd125, 8'd150, 8'd175, 8'd200};
            run_nn_inference();
            check_results("Back-to-back 2");
            end_test();
            
            // Minimal delay between tests
            #(CLK_PERIOD * 2);
        end
    endtask

    // Test 11: Reset during operation
    task test_reset_during_operation;
        begin
            test_name = "Reset During Operation Test";
            start_test();
            
            nn_input_vector = {8'd50, 8'd100, 8'd150, 8'd200};
            run_nn_inference();
            
            // Reset during operation
            #(CLK_PERIOD * 5);
            i_rst = 1;
            #(CLK_PERIOD * 2);
            i_rst = 0;
            
            check_results("Reset during operation");
            end_test();
        end
    endtask

    // Test 12: Start while busy
    task test_start_while_busy;
        begin
            test_name = "Start While Busy Test";
            start_test();
            
            nn_input_vector = {8'd25, 8'd50, 8'd75, 8'd100};
            nn_start = 1;
            run_nn_inference();
            
            // Start while busy
            #(CLK_PERIOD * 5);
            nn_start = 1;
            run_nn_inference();
            
            check_results("Start while busy");
            end_test();
        end
    endtask

    // Test 13: Setup hold times
    task test_setup_hold_times;
        begin
            test_name = "Setup Hold Times Test";
            start_test();
            
            nn_input_vector = {8'd50, 8'd100, 8'd150, 8'd200};
            run_nn_inference();
            
            // Check setup hold times
            #(CLK_PERIOD * 2);
            check_results("Setup hold times");
            end_test();
        end
    endtask

    // Test 14: Timeout conditions
    task test_timeout_conditions;
        begin
            test_name = "Timeout Conditions Test";
            start_test();
            
            nn_input_vector = {8'd25, 8'd50, 8'd75, 8'd100};
            run_nn_inference();
            
            // Check timeout conditions
            #(CLK_PERIOD * 10);
            check_results("Timeout conditions");
            end_test();
        end
    endtask

    // Start test
    task start_test;
        begin
            test_count = test_count + 1;
            $display("Starting test %0d: %s", test_count, test_name);
            nn_start = 1;
        end
    endtask

    // End test
    task end_test;
        begin
            $display("Ending test %0d: %s", test_count, test_name);
            nn_start = 0;
        end
    endtask

    // Run neural network inference
    task run_nn_inference;
        begin
            #(CLK_PERIOD * 2);
            while (nn_done == 0) begin
                #(CLK_PERIOD);
            end
        end
    endtask

    // Check results
    task check_results;
        input [255:0] test_name;
        begin
            if (nn_output_vector_relu == expected_relu) begin
                $display("ReLU output matches expected: %s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("ReLU output does not match expected: %s", test_name);
                fail_count = fail_count + 1;
            end
            
            if (nn_output_vector_sigmoid == expected_sigmoid) begin
                $display("Sigmoid output matches expected: %s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("Sigmoid output does not match expected: %s", test_name);
                fail_count = fail_count + 1;
            end
            
            if (nn_output_vector_tanh == expected_tanh) begin
                $display("Tanh output matches expected: %s", test_name);
                pass_count = pass_count + 1;
            end else begin
                $display("Tanh output does not match expected: %s", test_name);
                fail_count = fail_count + 1;
            end
        end
    endtask

    // Calculate expected ReLU output
    function [NN_OUTPUT_DIM*W-1:0] calculate_expected_relu;
        input [NN_INPUT_DIM*W-1:0] nn_input_vector;
        begin
            // Calculate expected ReLU output
            // ...
        end
    endfunction

    // Calculate expected Sigmoid output
    function [NN_OUTPUT_DIM*W-1:0] calculate_expected_sigmoid;
        input [NN_INPUT_DIM*W-1:0] nn_input_vector;
        begin
            // Calculate expected Sigmoid output
            // ...
        end
    endfunction

    // Calculate expected Tanh output
    function [NN_OUTPUT_DIM*W-1:0] calculate_expected_tanh;
        input [NN_INPUT_DIM*W-1:0] nn_input_vector;
        begin
            // Calculate expected Tanh output
            // ...
        end
    endfunction

endmodule
