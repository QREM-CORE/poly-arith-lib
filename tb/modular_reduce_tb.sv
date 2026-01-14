// ==========================================================
// Testbench for Table-Based Modular Reduction (Pipelined)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - Inha Architecture (Updated 26-bit)
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
    logic [25:0]    product_i; // UPDATED: 26-bit input
    logic           valid_o;
    logic [11:0]    result_o;

    // Verification Stats
    int error_count = 0;
    int sent_count  = 0;
    int recv_count  = 0;

    // ------------------------------------------------------
    // Scoreboard (Queue for Pipelined Checking)
    // ------------------------------------------------------
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
    // UPDATED: Input width 26 bits
    function automatic logic [11:0] get_expected(input logic [25:0] val);
        return val % 3329;
    endfunction

    // ------------------------------------------------------
    // Task: Drive Single Input
    // ------------------------------------------------------
    // UPDATED: Input width 26 bits
    task automatic drive_input(input logic [25:0] val);
        @(posedge clk);
        valid_i     <= 1'b1;
        product_i   <= val;

        // Calculate expected result and push to scoreboard
        expected_queue.push_back(get_expected(val));
        sent_count++;
    endtask

    // ------------------------------------------------------
    // Monitor Process
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
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting Modular Reducer Verification (26-bit Extended)");
        $display("==========================================================");

        // -------------------------
        // Test 1: Zero & Unity
        // -------------------------
        drive_input(26'd0);
        drive_input(26'd1);

        // -------------------------
        // Test 2: Powers of 2 (LUT Boundaries)
        // -------------------------
        drive_input(26'd4096);      // 2^12 (LUT 15:12)
        drive_input(26'd65536);     // 2^16 (LUT 19:16)
        drive_input(26'd1048576);   // 2^20 (LUT 23:20)

        // Testing the Extra Bits (Karatsuba Overflow Range)
        drive_input(26'd16777216);  // 2^24
        drive_input(26'd33554432);  // 2^25

        // -------------------------
        // Test 3: The "Black Swan" Case (Fixed)
        // -------------------------
        // Input 0x1DDFFF = 1,957,887.
        drive_input(26'h1DDFFF);

        // -------------------------
        // Test 4: Maximum Karatsuba Sum
        // -------------------------
        // (3328+3328)*(3328+3328) = 6656^2 = 44,302,336
        drive_input(26'd44302336);

        // -------------------------
        // Test 5: Exact Multiples of Q
        // -------------------------
        drive_input(26'd3329);
        drive_input(26'd332900);
        drive_input(26'd3329000);

        // -------------------------
        // Test 6: Pipeline Stress Test (Randomized)
        // -------------------------
        $display("Starting Randomized Regression (Karatsuba Range)...");

        repeat(10000) begin
            logic [31:0] rand32;
            logic [25:0] rand_val;

            rand32 = $urandom();
            rand_val = rand32[25:0]; // Slice to 26 bits

            drive_input(rand_val);
        end

        // End of Input Stream
        @(posedge clk);
        valid_i = 0;

        // Wait for pipeline to drain
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
