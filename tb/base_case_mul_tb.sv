// ==========================================================
// Testbench for Base Case Multiplier (Enhanced Debugging)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - 2-Cycle Latency
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module base_case_mul_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;
    logic           valid_i;

    // Inputs
    coeff_t a0, a1;
    coeff_t b0, b1;
    coeff_t zeta;

    // Outputs
    coeff_t c0, c1;
    logic   valid_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue with Metadata)
    // ------------------------------------------------------
    typedef struct {
        coeff_t c0;
        coeff_t c1;
        // Debug info to track which test this expectation belongs to
        string  test_name;
        coeff_t orig_a0, orig_a1;
        coeff_t orig_b0, orig_b1;
        coeff_t orig_zeta;
    } expected_t;

    expected_t expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    base_case_mul dut (
        .clk(clk),
        .rst(rst),
        .valid_i(valid_i),
        .a0_i(a0), .a1_i(a1),
        .b0_i(b0), .b1_i(b1),
        .zeta_i(zeta),
        .c0_o(c0), .c1_o(c1),
        .valid_o(valid_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ------------------------------------------------------
    // Golden Model (Pure Math)
    // ------------------------------------------------------
    function automatic expected_t get_expected(
        input coeff_t in_a0, input coeff_t in_a1,
        input coeff_t in_b0, input coeff_t in_b1,
        input coeff_t in_zeta,
        input string  name
    );
        logic signed [63:0] term_a0b0, term_a1b1, term_a0b1, term_a1b0;
        logic signed [63:0] c0_full, c1_full;
        expected_t res;

        // Save Debug Metadata
        res.test_name = name;
        res.orig_a0 = in_a0; res.orig_a1 = in_a1;
        res.orig_b0 = in_b0; res.orig_b1 = in_b1;
        res.orig_zeta = in_zeta;

        // 1. Compute C0 = (a0*b0 + a1*b1*zeta) % Q
        term_a0b0 = in_a0 * in_b0;
        term_a1b1 = in_a1 * in_b1;

        c0_full = term_a0b0 + (term_a1b1 * in_zeta);
        res.c0  = c0_full % 3329;

        // 2. Compute C1 = (a0*b1 + a1*b0) % Q
        term_a0b1 = in_a0 * in_b1;
        term_a1b0 = in_a1 * in_b0;

        c1_full = term_a0b1 + term_a1b0;
        res.c1  = c1_full % 3329;

        return res;
    endfunction

    // ------------------------------------------------------
    // Driver Task
    // ------------------------------------------------------
    task automatic drive_input(
        input coeff_t in_a0, input coeff_t in_a1,
        input coeff_t in_b0, input coeff_t in_b1,
        input coeff_t in_zeta,
        input string  test_name
    );
        expected_t exp;

        // Drive Signals
        @(posedge clk);
        valid_i <= 1'b1;
        a0 <= in_a0; a1 <= in_a1;
        b0 <= in_b0; b1 <= in_b1;
        zeta <= in_zeta;

        // Calculate Expectation and Push to Queue
        exp = get_expected(in_a0, in_a1, in_b0, in_b1, in_zeta, test_name);
        expected_queue.push_back(exp);
        sent_count++;
    endtask

    // ------------------------------------------------------
    // Monitor Process (Checks output whenever valid_o is high)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            expected_t exp_pop;

            if (expected_queue.size() == 0) begin
                $error("[FAIL] Unexpected valid_o! Queue is empty.");
                error_count++;
            end else begin
                exp_pop = expected_queue.pop_front();

                if (c0 !== exp_pop.c0 || c1 !== exp_pop.c1) begin
                    $error("==================================================");
                    $error("[FAIL] Test Case: %s", exp_pop.test_name);
                    $error("--------------------------------------------------");
                    $error("INPUTS: A={%0d, %0d} | B={%0d, %0d} | Zeta=%0d",
                            exp_pop.orig_a0, exp_pop.orig_a1,
                            exp_pop.orig_b0, exp_pop.orig_b1,
                            exp_pop.orig_zeta);
                    $error("EXPECTED: C0=%0d, C1=%0d", exp_pop.c0, exp_pop.c1);
                    $error("RECEIVED: C0=%0d, C1=%0d", c0, c1);
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
        // Init
        rst = 1;
        valid_i = 0;
        a0 = 0; a1 = 0; b0 = 0; b1 = 0; zeta = 0;

        // Reset Sequence
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting Base Case Multiplier Verification (Explicit Mode)");
        $display("==========================================================");

        // -------------------------
        // Test 1: Zeros
        // -------------------------
        drive_input(0,0,0,0,0, "Test 1: All Zeros");

        // -------------------------
        // Test 2: Identity
        // -------------------------
        drive_input(1,0,1,0, 100, "Test 2: Identity (1*1)");

        // -------------------------
        // Test 3: Zeta Gen
        // -------------------------
        drive_input(0,1,0,1, 50, "Test 3: X*X = Zeta");

        // -------------------------
        // Test 4: Maximum Coefficients (The "Karatsuba Stress Test")
        // -------------------------
        // Inputs = 3328 (Max valid value).
        // This forces the Middle Term sum to ~44 million, testing the 26-bit expansion.
        drive_input(3328, 3328, 3328, 3328, 1, "Test 4: Max Coeffs");

        // -------------------------
        // Test 5: Maximum Zeta
        // -------------------------
        // This stresses the C0 calculation: (a1*b1*zeta)
        // We want to make sure the multiplication by a large Zeta doesn't overflow or glitch.
        drive_input(100, 3328, 100, 3328, 3328, "Test 5: Max Zeta");

        // -------------------------
        // Test 6: The "Cross-Term" Stress
        // -------------------------
        // We want to generate a huge C1 result (a0*b1 + a1*b0)
        // Set a0=Max, b1=Max, a1=Max, b0=Max.
        // This specifically targets the logic: P_sum - P_high - P_low
        drive_input(3328, 3328, 3328, 3328, 10, "Test 6: Cross Term Stress");

        // -------------------------
        // Test 7: The "Wrap Around" (Modulo Boundary)
        // -------------------------
        // Inputs that are exactly Q-1. The result usually wraps close to 0 or Q.
        drive_input(3328, 1, 1, 3328, 17, "Test 7: Boundary Wrap");

        // -------------------------
        // Test 8: Pipeline Stress (Random)
        // -------------------------
        $display("Stress Testing Pipeline...");

        for (int i=0; i < 10000; i++) begin
            string test_id;
            // Create a unique name for this iteration: "Random #0", "Random #1", etc.
            $sformat(test_id, "Random #%0d", i);

            drive_input(
                $urandom_range(0, 3328), $urandom_range(0, 3328),
                $urandom_range(0, 3328), $urandom_range(0, 3328),
                $urandom_range(0, 3328),
                test_id
            );
        end

        // Stop Driving
        @(posedge clk);
        valid_i = 0;

        // Wait for Pipeline to Drain (Queue empty)
        wait(expected_queue.size() == 0);
        repeat(5) @(posedge clk);

        // -------------------------
        // Final Report
        // -------------------------
        $display("==========================================================");
        if (error_count == 0) begin
            $display("ALL TESTS PASSED");
            $display("Vectors Sent: %0d", sent_count);
            $display("Vectors Recv: %0d", recv_count);
        end else begin
            $display("TEST FAILED: %0d Errors Found", error_count);
        end
        $display("==========================================================");
        $finish;
    end

endmodule
