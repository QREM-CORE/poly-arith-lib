// ==========================================================
// Testbench for Table-Based Modular Reduction (Pipelined)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - Inha Architecture
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module modular_reduce_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // DUT Interface
    logic           valid_i;
    logic [23:0]    product_i;
    logic           valid_o;
    logic [11:0]    result_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Pipelined Checking)
    // ------------------------------------------------------
    // We push expected values here when valid_i=1
    // We pop and check when valid_o=1
    logic [11:0] expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    modular_reduce dut (
        .clk(clk),
        .rst(rst),
        .valid_i(valid_i),
        .product_i(product_i),
        .valid_o(valid_o),
        .result_o(result_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz Clock
    end

    // ------------------------------------------------------
    // Golden Model (Standard Modulo)
    // ------------------------------------------------------
    function automatic logic [11:0] get_expected(input logic [23:0] val);
        // The hardware calculates: val % 3329
        // Note: The hardware output is strictly [0, 3328]
        return val % 3329;
    endfunction

    // ------------------------------------------------------
    // Task: Drive Single Input
    // ------------------------------------------------------
    task automatic drive_input(input logic [23:0] val);
        @(posedge clk);
        valid_i     <= 1'b1;
        product_i   <= val;

        // Calculate expected result and push to scoreboard
        expected_queue.push_back(get_expected(val));
        sent_count++;
    endtask

    // ------------------------------------------------------
    // Monitor Process (Checks output whenever valid_o is high)
    // ------------------------------------------------------
    always @(posedge clk) begin
        if (valid_o) begin
            logic [11:0] expected_val;

            if (expected_queue.size() == 0) begin
                $error("[FAIL] Unexpected output! Queue empty.");
                error_count++;
            end else begin
                expected_val = expected_queue.pop_front();

                if (result_o !== expected_val) begin
                    $error("[FAIL] Mismatch! Recv: %0d | Exp: %0d", result_o, expected_val);
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
        product_i = 0;

        // Reset Sequence
        repeat(5) @(posedge clk);
        rst = 0; // Release Reset (Active High in your module)
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Reducer Verification (Table-Based)");
        $display("==========================================================");

        // -------------------------
        // Test 1: Zero & Unity
        // -------------------------
        drive_input(24'd0);
        drive_input(24'd1);

        // -------------------------
        // Test 2: Powers of 2 (LUT Boundaries)
        // -------------------------
        // Testing specific bits that trigger LUT entries
        drive_input(24'd4096);      // 2^12 (LUT 15:12)
        drive_input(24'd65536);     // 2^16 (LUT 19:16)
        drive_input(24'd1048576);   // 2^20 (LUT 23:20)

        // -------------------------
        // Test 3: The "Black Swan" Case
        // -------------------------
        // Input 0x1DDFFF = 1,957,887.
        // This triggers the Max Sum (13751) -> Requires 4Q Subtraction.
        // Expected: 1957887 % 3329 = 435.
        drive_input(24'h1DDFFF);

        // -------------------------
        // Test 4: Typical Poly Multiply Max
        // -------------------------
        // 3328 * 3328 = 11,075,584
        drive_input(24'd11075584);

        // -------------------------
        // Test 5: Exact Multiples of Q (Result should be 0)
        // -------------------------
        drive_input(24'd3329);
        drive_input(24'd6658);
        drive_input(24'd332900);

        // -------------------------
        // Test 6: Pipeline Stress Test (Randomized)
        // -------------------------
        $display("Starting Randomized Regression (Real System Constraints)...");

        repeat(10000) begin
            logic [23:0] rand_val;

            // 1. Generate 32-bit random
            rand_val = $urandom();

            // 2. Constrain to the REAL maximum product of ML-KEM.
            // Max coeff is 3328. Max product is 3328 * 3328 = 11,075,584.
            // If we exceed this, we force it back into range.
            if (rand_val > 11075584) begin
                rand_val = rand_val % 11075584;
            end

            drive_input(rand_val);
        end

        // End of Input Stream
        @(posedge clk);
        valid_i = 0;

        // Wait for pipeline to drain (Queue should empty)
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
