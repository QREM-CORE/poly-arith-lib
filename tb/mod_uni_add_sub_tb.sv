// ==========================================================
// Testbench for Modular Unified Adder/Subtractor
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - 12-bit Modular Arithmetic
// Verified Latency: 2 Clock Cycles
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module mod_uni_add_sub_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         op1;
    coeff_t         op2;
    logic           is_sub; // 0 = Add, 1 = Sub
    logic           valid_i;

    // Outputs
    coeff_t         result_o;
    logic           valid_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Pipelined Checking)
    // ------------------------------------------------------
    // Stores expected results to compare against valid_o
    coeff_t expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    mod_uni_add_sub dut (
        .clk(clk),
        .rst(rst),
        .op1_i(op1),
        .op2_i(op2),
        .is_sub_i(is_sub),
        .valid_i(valid_i),
        .result_o(result_o),
        .valid_o(valid_o)
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
    // Calculates (A +/- B) % 3329 safely
    function automatic coeff_t get_expected(input coeff_t a, input coeff_t b, input logic sub);
        longint res;
        if (sub) begin
            // Subtraction: Ensure positive result by adding Q before modulo
            // (a - b) mod Q
            res = (longint'(a) + 3329 - longint'(b)) % 3329;
        end else begin
            // Addition: (a + b) mod Q
            res = (longint'(a) + longint'(b)) % 3329;
        end
        return coeff_t'(res);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Inputs
    // ------------------------------------------------------
    task automatic drive_input(input coeff_t a, input coeff_t b, input logic sub, input string name);
        @(posedge clk);
        valid_i <= 1'b1;
        op1     <= a;
        op2     <= b;
        is_sub  <= sub;

        // Push expected result to queue
        expected_queue.push_back(get_expected(a, b, sub));
        sent_count++;
    endtask

    // ------------------------------------------------------
    // Monitor Process
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            coeff_t expected_val;

            if (expected_queue.size() == 0) begin
                $error("[FAIL] Unexpected output valid_o! Queue is empty.");
                error_count++;
            end else begin
                expected_val = expected_queue.pop_front();

                if (result_o !== expected_val) begin
                    $error("==================================================");
                    $error("[FAIL] Mismatch!");
                    $error("Expected: %0d", expected_val);
                    $error("Received: %0d", result_o);
                    $error("==================================================");
                    error_count++;
                end else begin
                    recv_count++;
                end
            end
        end
    end

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize
        rst = 1;
        valid_i = 0;
        op1 = 0;
        op2 = 0;
        is_sub = 0;

        // Reset Sequence
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Adder/Subtractor Verification");
        $display("==========================================================");

        // --------------------------------------------------
        // 1. ADDITION Corner Cases
        // --------------------------------------------------
        // Standard
        drive_input(0, 0,       0, "Add: Zero + Zero");
        drive_input(100, 200,   0, "Add: Standard");

        // Overflow / Reduction Check
        // 3328 + 1 = 3329 -> 0
        drive_input(3328, 1,    0, "Add: Boundary (result 0)");
        // 3328 + 3328 = 6656 -> 3327
        drive_input(3328, 3328, 0, "Add: Max + Max");

        // --------------------------------------------------
        // 2. SUBTRACTION Corner Cases
        // --------------------------------------------------
        // Standard
        drive_input(500, 200,   1, "Sub: Standard");
        drive_input(500, 500,   1, "Sub: Result Zero");

        // Underflow / Wrap Check
        // 0 - 1 = -1 -> 3328
        drive_input(0, 1,       1, "Sub: Underflow 1");
        // 10 - 20 = -10 -> 3319
        drive_input(10, 20,     1, "Sub: Underflow Standard");

        // --------------------------------------------------
        // 3. Random Stress Testing
        // --------------------------------------------------
        $display("Starting Random Stress Loop (500 vectors)...");

        for (int i = 0; i < 500; i++) begin
            coeff_t rand_a, rand_b;
            logic   rand_sub;

            rand_a   = $urandom_range(0, 3328);
            rand_b   = $urandom_range(0, 3328);
            rand_sub = $urandom_range(0, 1); // Randomly pick Add or Sub

            drive_input(rand_a, rand_b, rand_sub, "Random");
        end

        // Stop Driving
        @(posedge clk);
        valid_i = 0;
        op1 = 0; op2 = 0; is_sub = 0;

        // Wait for Pipeline to Drain
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
        end
        $display("==========================================================");
        $finish;
    end

endmodule
