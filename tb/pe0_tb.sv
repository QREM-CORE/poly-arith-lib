// ==========================================================
// Testbench for PE0 (Butterfly Unit)
// Author: Kiet Le
// Target: FIPS 203 (ML-KEM) - Mixed Radix Butterfly
// Mode: Sequential Verification (One input at a time)
// ==========================================================
`timescale 1ns/1ps

import poly_arith_pkg::*;

module pe0_tb();

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic           clk;
    logic           rst;

    // Inputs
    coeff_t         a0_i;
    coeff_t         b0_i;
    coeff_t         w0_i;
    logic [3:0]     ctrl_i;
    logic           valid_i;

    // Outputs
    coeff_t         u0_o;
    coeff_t         v0_o;
    logic           valid_o;

    // Verification Stats
    int error_count = 0;
    int pass_count  = 0;

    // ------------------------------------------------------
    // Constants
    // ------------------------------------------------------
    localparam logic [3:0] MODE_NTT  = 4'b1010; // Decimal 10
    localparam logic [3:0] MODE_INTT = 4'b1111; // Decimal 15
    localparam int         MODULUS   = 3329;

    // ------------------------------------------------------
    // DUT Instantiation
    // ------------------------------------------------------
    pe0 dut (
        .clk(clk),
        .rst(rst),
        .a0_i(a0_i),
        .b0_i(b0_i),
        .w0_i(w0_i),
        .ctrl_i(ctrl_i),
        .valid_i(valid_i),
        .u0_o(u0_o),
        .v0_o(v0_o),
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
    // Golden Model Helpers
    // ------------------------------------------------------
    function automatic coeff_t mod_add(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) + longint'(b)) % MODULUS);
    endfunction

    function automatic coeff_t mod_sub(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) - longint'(b) + MODULUS) % MODULUS);
    endfunction

    function automatic coeff_t mod_mul(input coeff_t a, input coeff_t b);
        return coeff_t'((longint'(a) * longint'(b)) % MODULUS);
    endfunction

    function automatic coeff_t mod_div2(input coeff_t a);
        longint val = longint'(a);
        if (val % 2 != 0) val = val + MODULUS;
        return coeff_t'(val / 2);
    endfunction

    // ------------------------------------------------------
    // Task: Drive Input & Check Result Sequentially
    // ------------------------------------------------------
    task automatic drive_verify(
        input coeff_t a,
        input coeff_t b,
        input coeff_t w,
        input logic [3:0] mode,
        input string name
    );
        coeff_t exp_u, exp_v;
        coeff_t t;

        // 1. Calculate Expected Values based on Mode (Gold Model)
        if (mode == MODE_NTT) begin
            // NTT: U = A + BW, V = A - BW
            t = mod_mul(b, w);
            exp_u = mod_add(a, t);
            exp_v = mod_sub(a, t);
        end else begin
            // INTT: U = (A+B)/2, V = (A-B)*W
            exp_u = mod_div2(mod_add(a, b));
            t     = mod_sub(a, b);
            exp_v = mod_mul(t, w);
        end

        // 2. Drive Inputs
        @(posedge clk);
        valid_i <= 1'b1;
        a0_i    <= a;
        b0_i    <= b;
        w0_i    <= w;
        ctrl_i  <= mode;

        // 3. Deassert valid after one cycle (Pulse input)
        @(posedge clk);
        valid_i <= 1'b0;
        // Clear inputs to prevent latch inference or confusion
        a0_i <= 0; b0_i <= 0; w0_i <= 0;

        // 4. Wait for Result (Blocking)
        // This makes the testbench latency-agnostic (works for 3 or 4 cycles)
        wait(valid_o == 1'b1);

        // 5. Compare Results
        if (u0_o !== exp_u || v0_o !== exp_v) begin
            $display("==================================================");
            $display("[FAIL] %s", name);
            $display("Mode: %s", (mode == MODE_NTT) ? "NTT" : "INTT");
            $display("Ctrl: %0d", mode);
            $display("Inputs: A=%0d, B=%0d, W=%0d", a, b, w);
            if (u0_o !== exp_u) $display("   U-Mismatch! Exp: %0d, Got: %0d", exp_u, u0_o);
            if (v0_o !== exp_v) $display("   V-Mismatch! Exp: %0d, Got: %0d", exp_v, v0_o);
            $display("==================================================");
            error_count++;
        end else begin
            pass_count++;
        end

        // 6. Wait for valid_o to drop before next test
        @(posedge clk);

    endtask

    // ==========================================================
    // Main Test Procedure
    // ==========================================================
    initial begin
        // Initialize
        rst = 1;
        valid_i = 0;
        a0_i = 0; b0_i = 0; w0_i = 0;
        ctrl_i = 0;

        // Reset Sequence
        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE0 Verification (Sequential Mode)");
        $display("==========================================================");

        // --------------------------------------------------
        // 1. NTT Mode Tests
        // --------------------------------------------------
        $display("--- Testing NTT Mode ---");
        drive_verify(0, 0, 0, MODE_NTT, "NTT Zero");
        drive_verify(10, 2, 5, MODE_NTT, "NTT Simple (10, 2, 5)");
        drive_verify(0, 1, 3328, MODE_NTT, "NTT Neg Wrap");

        // --------------------------------------------------
        // 2. INTT Mode Tests
        // --------------------------------------------------
        $display("--- Testing INTT Mode ---");
        drive_verify(2, 0, 1, MODE_INTT, "INTT Div2 (2,0)");
        drive_verify(1, 0, 1, MODE_INTT, "INTT Odd Div2 (1,0)");
        drive_verify(20, 10, 2, MODE_INTT, "INTT Path (20,10,2)");

        // --------------------------------------------------
        // 3. Random Stress Loop
        // --------------------------------------------------
        $display("--- Starting Random Stress Loop (500 vectors) ---");

        for (int i = 0; i < 500; i++) begin
            coeff_t ra, rb, rw;
            logic [3:0] rmode;

            ra = $urandom_range(0, 3328);
            rb = $urandom_range(0, 3328);
            rw = $urandom_range(0, 3328);
            rmode = (i % 2 == 0) ? MODE_NTT : MODE_INTT;

            drive_verify(ra, rb, rw, rmode, "Random Vector");
        end

        // --------------------------------------------------
        // Final Summary Report
        // --------------------------------------------------
        repeat(5) @(posedge clk);
        $display("");
        $display("==========================================================");
        $display("SUMMARY");
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", error_count);
        $display("----------------------------------------------------------");

        if (error_count == 0) begin
            $display("RESULT: SUCCESS");
        end else begin
            $display("RESULT: FAILURE");
        end
        $display("==========================================================");
        $finish;
    end

endmodule
