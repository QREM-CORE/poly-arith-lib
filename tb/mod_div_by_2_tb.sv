// ==========================================================
// Testbench for Modular Division by 2
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - 12-bit Modular Arithmetic
// Verified Latency: 0 Clock Cycles (Combinational)
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module mod_div_by_2_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;

    // Inputs
    coeff_t         op1;

    // Outputs
    coeff_t         result_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Checking)
    // ------------------------------------------------------
    // We store a struct so we can recall the input that generated the expected value
    typedef struct {
        coeff_t input_val;
        coeff_t expected_val;
        string  name;
    } transaction_t;

    transaction_t expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    mod_div_by_2 dut (
        .op_i(op1),
        .op_o(result_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock
    end

    // ------------------------------------------------------
    // Golden Model (Pure Math)
    // ------------------------------------------------------
    // The modular inverse of 2 mod 3329 is 1665.
    function automatic coeff_t get_expected(input coeff_t a);
        longint product;
        product = longint'(a) * 1665; // Multiply by modular inverse
        return coeff_t'(product % 3329);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Inputs
    // ------------------------------------------------------
    task automatic drive_input(input coeff_t a, input string name);
        transaction_t trans;

        // Drive input (Non-blocking update happens at Posedge)
        op1 <= a;

        // Prepare Transaction
        trans.input_val = a;
        trans.expected_val = get_expected(a);
        trans.name = name;

        // Push to queue
        expected_queue.push_back(trans);
        sent_count++;

        // Wait for next cycle
        @(posedge clk);
    endtask

    // ------------------------------------------------------
    // Monitor Process
    // ------------------------------------------------------
    always @(negedge clk) begin
        if (expected_queue.size() > 0) begin
            transaction_t trans;
            trans = expected_queue.pop_front();

            if (result_o !== trans.expected_val) begin
                $display("\n==================================================");
                $display("[FAIL] Mismatch on %s", trans.name);
                $display("  Input:    %0d", trans.input_val);
                $display("  Expected: %0d", trans.expected_val);
                $display("  Received: %0d", result_o);
                $display("==================================================\n");

                // Use ONE $error to flag the simulator without spamming metadata
                $error("Test Failed");
                error_count++;
            end else begin
                // Optional: Uncomment for "quiet" success logging
                // $display("[PASS] Input: %0d | Output: %0d", trans.input_val, result_o);
                recv_count++;
            end
        end
    end

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize
        op1 = 0;

        // Wait for startup
        repeat(5) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Divider Verification (Combinational)");
        $display("==========================================================");

        // --------------------------------------------------
        // 1. Corner Cases
        // --------------------------------------------------
        drive_input(0, "Zero");
        drive_input(1, "One");
        drive_input(2, "Two");
        drive_input(3328, "Max Value");
        drive_input(3327, "Max Odd");

        // --------------------------------------------------
        // 2. Random Stress Testing
        // --------------------------------------------------
        $display("Starting Random Stress Loop (500 vectors)...");

        for (int i = 0; i < 500; i++) begin
            coeff_t rand_a;
            rand_a = $urandom_range(0, 3328);
            drive_input(rand_a, "Random");
        end

        // Wait for Queue to Drain
        wait(expected_queue.size() == 0);
        repeat(5) @(posedge clk);

        // --------------------------------------------------
        // Final Report
        // --------------------------------------------------
        $display("==========================================================");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED");
            $display("Vectors Processed: %0d", recv_count);
        end else begin
            $display("TEST FAILED: %0d Errors Found", error_count);
            $fatal(1, "mod_div_by_2_tb: Testbench failed.");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
