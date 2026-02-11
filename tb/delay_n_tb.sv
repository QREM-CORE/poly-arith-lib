// ==========================================================
// Testbench for Delay Line (Shift Register)
// Description: Verifies that data input appears at output
//              exactly DEPTH cycles later.
// ==========================================================
`timescale 1ns/1ps

module delay_n_tb;

    // ------------------------------------------------------
    // Parameters & Signals
    // ------------------------------------------------------
    localparam int DWIDTH = 12;
    localparam int DEPTH = 3; // Matching the PE0 diagram

    logic              clk;
    logic              rst;
    logic [DWIDTH-1:0] data_i;
    logic [DWIDTH-1:0] data_o;

    // ------------------------------------------------------
    // Verification Stats
    // ------------------------------------------------------
    int error_count = 0;

    // Queue to store expected values
    logic [DWIDTH-1:0] expected_queue [$];

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    delay_n #(
        .DWIDTH(DWIDTH),
        .DEPTH(DEPTH)
    ) dut (
        .clk    (clk),
        .rst    (rst),
        .data_i (data_i),
        .data_o (data_o)
    );

    // ------------------------------------------------------
    // Clock Generation
    // ------------------------------------------------------
    initial begin
        clk = 0;
        forever #5 clk = ~clk; // 100MHz
    end

    // ------------------------------------------------------
    // Test Procedure
    // ------------------------------------------------------
    initial begin
        $display("==================================================");
        $display("Testing Delay Module (Depth: %0d)", DEPTH);
        $display("==================================================");

        // 1. Initialize & Reset
        rst = 1;
        data_i = '0;

        // Pre-fill expectation for the reset cycles
        repeat(DEPTH) expected_queue.push_back(0);

        repeat(2) @(posedge clk);
        rst = 0;
        $display("[INFO] Reset released.");

        // 2. Drive Random Data
        repeat(20) begin
            @(posedge clk);
            // Push the *current* value which the DUT samples this edge
            expected_queue.push_back(data_i);

            // Drive the *new* value for the next edge
            data_i <= $urandom_range(0, 4095);
        end

        // 3. Wait for pipeline to drain
        repeat(DEPTH + 2) @(posedge clk);

        // 4. Final Report
        $display("==================================================");
        if (error_count == 0)
            $display("ALL TESTS PASSED");
        else 
            $display("FAILED: %0d Errors Found", error_count);
        $display("==================================================");
        $finish;
    end

    // ------------------------------------------------------
    // Monitor / Checker (Checks every cycle)
    // ------------------------------------------------------
    always @(negedge clk) begin
        if (!rst && expected_queue.size() > 0) begin
            logic [DWIDTH-1:0] expected;

            // Pop the value that should be coming out NOW
            expected = expected_queue.pop_front();

            if (data_o !== expected) begin
                $display("[FAIL] Time: %0t | Exp: %0d | Recv: %0d", $time, expected, data_o);
                error_count++;
            end
            // Removed 'else' block to silence passing checks
        end
    end

endmodule
