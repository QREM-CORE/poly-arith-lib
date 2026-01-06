// ==========================================================
// Testbench for Modular Reduce (Montgomery)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM)
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*; // Import constants like Q, Q_INV_NEG

module modular_reduce_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic signed [31:0] z;      // Input (Product)
    logic signed [15:0] res;    // Output (Reduced)

    // For verification stats
    int error_count = 0;
    int test_count = 0;

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    modular_reduce dut (
        .z_i(z),
        .res_o(res)
    );

    // ------------------------------------------------------
    // Golden Model (Software implementation of the logic)
    // ------------------------------------------------------
    function automatic logic signed [15:0] expected_montgomery(input logic signed [31:0] z_val);
        logic signed [15:0] m_gold;
        logic signed [31:0] t_gold;

        // 1. Calculate reduction factor
        m_gold = 16'(z_val * 16'(Q_INV_NEG));

        // 2. Perform reduction (z + m*q) / 2^16
        // Note: In SW, we divide. In HW, we slice [31:16].
        // We use >>> 16 to mimic the signed slice behavior perfectly.
        t_gold = (z_val + (m_gold * 32'(Q))) >>> 16;

        return t_gold[15:0];
    endfunction

    // ------------------------------------------------------
    // Helper Task: Check Result
    // ------------------------------------------------------
    task automatic check_result(input string test_name);
        logic signed [15:0] exp;
        exp = expected_montgomery(z);

        // Allow for 1ps delta delay for logic to settle
        #1;

        if (res !== exp) begin
            $error("[FAIL] %s | Input: %0d | Exp: %0d | Got: %0d",
                   test_name, z, exp, res);
            error_count++;
        end else begin
            // Uncomment for verbose logging
            // $display("[PASS] %s | Input: %0d | Result: %0d", test_name, z, res);
        end
        test_count++;
    endtask

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        $display("==========================================================");
        $display("Starting Modular Reducer Verification");
        $display("==========================================================");

        // -------------------------
        // Test 1: The Zero Case
        // -------------------------
        z = 0;
        check_result("Zero Input");

        // -------------------------
        // Test 2: Max Valid Positive Input
        // Montgomery Limit: Z < Q * R = 3329 * 65536 = 218,169,344
        // -------------------------
        z = 218169343; // Just below the limit
        check_result("Max Positive Valid");

        // -------------------------
        // Test 3: Max Valid Negative Input
        // Montgomery Limit: Z > -Q * R
        // -------------------------
        z = -218169343; // Just above the limit
        check_result("Max Negative Valid");

        // -------------------------
        // Test 4: Typical Poly Multiply Case (Max Coeffs)
        // 3328 * 3328 (Standard * Standard)
        // -------------------------
        z = 3328 * 3328;
        check_result("Max Coeff Product");

        // -------------------------
        // Test 5: Negative Coeff Product (CBD Noise)
        // -2 * 3328
        // -------------------------
        z = -2 * 3328;
        check_result("Negative Coeff Product");

        // -------------------------
        // Test 6: One times One
        // -------------------------
        z = 1;
        check_result("Unity Input");

        // -------------------------
        // Test 7: The "Trap" Case (Input = Q)
        // Checking if reduction handles inputs that are multiples of Q
        // -------------------------
        z = 3329;
        check_result("Input equals Q");

        z = 3329 * 100;
        check_result("Input equals 100*Q");

        // -------------------------
        // Test 8: Randomized Regression
        // -------------------------
        $display("Starting Randomized Regression (10,000 vectors)...");
        for (int i = 0; i < 10000; i++) begin
            // Generate random 32-bit signed integer
            z = $urandom();

            // Constrain z to valid Montgomery range for this unit (-Q*R to Q*R)
            // Range is roughly +/- 218 million.
            // We use modulo to keep it in range for meaningful testing.
            if (z > 218169000) z = z % 218169000;
            if (z < -218169000) z = z % 218169000;

            check_result("Random");
        end

        // -------------------------
        // Final Report
        // -------------------------
        $display("==========================================================");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED (%0d Vectors Checked)", test_count);
        end else begin
            $display("TEST FAILED: %0d Errors Found", error_count);
        end
        $display("==========================================================");
        $finish;
    end

endmodule
