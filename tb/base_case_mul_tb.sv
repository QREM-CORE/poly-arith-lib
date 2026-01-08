// ==========================================================
// Testbench for Base Case Multiplier (Algorithm 12)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM)
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module base_case_mul_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    // Inputs
    coeff_t a0, a1;
    coeff_t b0, b1;
    coeff_t zeta;

    // Outputs
    coeff_t c0, c1;

    // Verification Stats
    int error_count = 0;
    int test_count = 0;

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    base_case_mul dut (
        .a0(a0), .a1(a1),
        .b0(b0), .b1(b1),
        .zeta(zeta),
        .c0(c0), .c1(c1)
    );

    // ------------------------------------------------------
    // Software Golden Model (Algorithm 12)
    // ------------------------------------------------------
    function automatic void expected_base_mul(
        input  coeff_t in_a0, input coeff_t in_a1,
        input  coeff_t in_b0, input coeff_t in_b1,
        input  coeff_t in_zeta,
        output coeff_t exp_c0, output coeff_t exp_c1
    );
        // --- 1. DECLARATIONS ---
        logic signed [31:0] term_a0b0, term_a1b1;
        logic signed [31:0] term_a0b1, term_a1b0;
        logic signed [15:0] mont_a1b1;
        logic signed [31:0] term_zeta;
        logic signed [31:0] c0_sum;
        logic signed [31:0] c1_sum;

        // --- 2. LOGIC  ---

        // --- Calculate c0: a0*b0 + a1*b1*zeta ---
        term_a0b0 = in_a0 * in_b0;
        term_a1b1 = in_a1 * in_b1;

        // Reduce the a1*b1 term first
        mont_a1b1 = sw_montgomery_reduce(term_a1b1);

        // Multiply by zeta
        term_zeta = mont_a1b1 * in_zeta;

        // Add a0*b0 and reduce final result
        c0_sum = term_a0b0 + term_zeta;
        exp_c0 = sw_montgomery_reduce(c0_sum);

        // --- Calculate c1: a0*b1 + a1*b0 ---
        term_a0b1 = in_a0 * in_b1;
        term_a1b0 = in_a1 * in_b0;

        c1_sum = term_a0b1 + term_a1b0;
        exp_c1 = sw_montgomery_reduce(c1_sum);

    endfunction

    // Helper: Software Montgomery Reduction
    function automatic logic signed [15:0] sw_montgomery_reduce(input logic signed [31:0] z_val);
        logic signed [15:0] m_gold;
        logic signed [31:0] t_gold;
        // m = z * Q_INV_NEG mod 2^16
        m_gold = 16'(z_val * 16'(Q_INV_NEG));
        // t = (z + m*q) / 2^16
        t_gold = (z_val + (m_gold * 32'(Q))) >>> 16;
        return t_gold[15:0];
    endfunction

    // ------------------------------------------------------
    // Helper Task: Check Result
    // ------------------------------------------------------
    task automatic check_result(input string test_name);
        coeff_t exp_c0, exp_c1;

        // Compute Expected Values
        expected_base_mul(a0, a1, b0, b1, zeta, exp_c0, exp_c1);

        // Allow time for DUT combinatorial logic
        #1;

        if (c0 !== exp_c0 || c1 !== exp_c1) begin
            $error("[FAIL] %s", test_name);
            $error("  Inputs: a0=%0d, a1=%0d, b0=%0d, b1=%0d, zeta=%0d", a0, a1, b0, b1, zeta);
            $error("  Exp c0: %0d | Got c0: %0d", exp_c0, c0);
            $error("  Exp c1: %0d | Got c1: %0d", exp_c1, c1);
            error_count++;
        end
        test_count++;
    endtask

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        $display("==========================================================");
        $display("Starting Base Case Multiplier Verification");
        $display("==========================================================");

        // -------------------------
        // Test 1: The All-Zero Case
        // -------------------------
        a0 = 0; a1 = 0; b0 = 0; b1 = 0; zeta = 0;
        check_result("All Zeros");

        // -------------------------
        // Test 2: Identity Multiplications
        // (1 + 0X) * (1 + 0X) = 1 + 0X
        // -------------------------
        a0 = 1; a1 = 0; b0 = 1; b1 = 0; zeta = 100; // Zeta shouldn't matter here
        check_result("Identity (1 * 1)");

        // -------------------------
        // Test 3: Simple Linear Term
        // (0 + 1X) * (0 + 1X) = X^2 = zeta
        // c0 should be zeta (modulo factors), c1 should be 0
        // -------------------------
        a0 = 0; a1 = 1; b0 = 0; b1 = 1; zeta = 50;
        // Note: Due to Montgomery factors (R^-1), exact output isn't just '50',
        // but the Golden Model will calculate the correct scaled value.
        check_result("X * X = Zeta");

        // -------------------------
        // Test 4: Mixed Terms
        // (1 + 1X) * (1 + 1X) = 1 + 2X + X^2 = (1 + zeta) + 2X
        // -------------------------
        a0 = 1; a1 = 1; b0 = 1; b1 = 1; zeta = 20;
        check_result("Binomial Square (1+X)^2");

        // -------------------------
        // Test 5: Negative Coefficients (CBD Noise)
        // Testing with typical small negative numbers
        // -------------------------
        a0 = -1; a1 = -2; b0 = 2; b1 = 1; zeta = 17;
        check_result("Small Negative Coeffs");

        // -------------------------
        // Test 6: Max Positive Values (Corner Case)
        // a, b = 3328 (approx Q-1)
        // -------------------------
        a0 = 3328; a1 = 3328; b0 = 3328; b1 = 3328; zeta = 17;
        check_result("Max Positive Inputs");

        // -------------------------
        // Test 7: Max Negative Values (Corner Case)
        // -------------------------
        a0 = -1664; a1 = -1664; b0 = -1664; b1 = -1664; zeta = 17;
        check_result("Max Negative Inputs");

        // -------------------------
        // Test 8: Large Zeta (Max Field Element)
        // -------------------------
        a0 = 10; a1 = 10; b0 = 10; b1 = 10; zeta = 3328;
        check_result("Large Zeta");

        // -------------------------
        // Test 9: Randomized Regression
        // -------------------------
        $display("Starting Randomized Regression (10,000 vectors)...");
        for (int i = 0; i < 10000; i++) begin
            // Randomize inputs within 12-bit signed range mostly,
            // but full 16-bit range is valid too.
            a0 = $urandom_range(0, 3328);
            // Occasionally flip sign to test negative inputs
            if ($urandom() % 2) a0 = -a0;

            a1 = $urandom_range(0, 3328); if ($urandom() % 2) a1 = -a1;
            b0 = $urandom_range(0, 3328); if ($urandom() % 2) b0 = -b0;
            b1 = $urandom_range(0, 3328); if ($urandom() % 2) b1 = -b1;

            // Zeta is strictly positive 0..3328 in our ROM, but stored in signed container
            zeta = $urandom_range(0, 3328);

            check_result("Random Vector");
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
