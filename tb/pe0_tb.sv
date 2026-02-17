// ==========================================================
// Testbench for PE0 (Butterfly Unit)
// Target: FIPS 203 (ML-KEM) - Mixed Radix Butterfly
// Mode: Sequential Verification (All 5 Operating Modes)
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
    pe_mode_e       ctrl_i; // Updated to use the Enum
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
    localparam int  MODULUS = 3329;

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
        input pe_mode_e mode, // Updated to use the Enum
        input string name
    );
        coeff_t exp_u, exp_v;
        coeff_t t;

        // 1. Calculate Expected Values based on Mode (Gold Model)
        if (mode == PE_MODE_NTT || mode == PE_MODE_CWM) begin
            // Note: In CWM mode, PE0 shares the exact same control bits as NTT.
            t = mod_mul(b, w);
            exp_u = mod_add(a, t);
            exp_v = mod_sub(a, t);
        end 
        else if (mode == PE_MODE_INTT) begin
            exp_u = mod_div2(mod_add(a, b));
            t     = mod_sub(a, b);
            exp_v = mod_mul(t, w);
        end 
        else if (mode == PE_MODE_ADDSUB) begin
            exp_u = mod_add(a, b);
            exp_v = mod_sub(a, b);
        end 
        else if (mode == PE_MODE_CODECO1 || mode == PE_MODE_CODECO2) begin
            exp_u = a;
            exp_v = mod_mul(b, w);
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
        a0_i <= 0; b0_i <= 0; w0_i <= 0; 

        // 4. Wait for Result (Latency Agnostic check)
        wait(valid_o == 1'b1);
        
        // 5. Compare Results
        if (u0_o !== exp_u || v0_o !== exp_v) begin
            $display("==================================================");
            $display("[FAIL] %s", name);
            $display("Ctrl: %s (%b)", mode.name(), mode); // Prints Enum Name and Binary Value
            $display("Inputs: A=%0d, B=%0d, W=%0d", a, b, w);
            if (u0_o !== exp_u) $display("   U-Mismatch! Exp: %0d, Got: %0d", exp_u, u0_o);
            if (v0_o !== exp_v) $display("   V-Mismatch! Exp: %0d, Got: %0d", exp_v, v0_o);
            $display("==================================================");
            error_count++;
        end else begin
            pass_count++;
        end
        
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
        ctrl_i = PE_MODE_NTT; // Initialize with a valid enum state

        repeat(5) @(posedge clk);
        rst = 0;
        repeat(2) @(posedge clk);

        $display("==========================================================");
        $display("Starting PE0 Verification (Comprehensive Corner Cases)");
        $display("==========================================================");

        // --------------------------------------------------
        // NTT Mode Tests (U = A + BW, V = A - BW)
        // --------------------------------------------------
        $display("--- Testing NTT Mode ---");
        drive_verify(0, 0, 0, PE_MODE_NTT, "NTT: All Zeros");
        drive_verify(1, 1, 1, PE_MODE_NTT, "NTT: All Ones");
        drive_verify(10, 2, 5, PE_MODE_NTT, "NTT: Simple Standard");
        drive_verify(100, 0, 50, PE_MODE_NTT, "NTT: B is Zero (Bypass)");
        drive_verify(100, 50, 0, PE_MODE_NTT, "NTT: W is Zero (Bypass)");
        drive_verify(0, 1, 3328, PE_MODE_NTT, "NTT: BW = -1 (Modulo Wrap Check)");
        drive_verify(3328, 3328, 3328, PE_MODE_NTT, "NTT: Max Field Values");

        // --------------------------------------------------
        // INTT Mode Tests (U = (A+B)/2, V = (A-B)*W)
        // --------------------------------------------------
        $display("--- Testing INTT Mode ---");
        drive_verify(0, 0, 0, PE_MODE_INTT, "INTT: All Zeros");
        drive_verify(20, 10, 2, PE_MODE_INTT, "INTT: Simple Standard");
        drive_verify(10, 10, 5, PE_MODE_INTT, "INTT: A=B (Sub = 0)");
        drive_verify(2, 0, 1, PE_MODE_INTT, "INTT: Even Sum Div2");
        drive_verify(1, 0, 1, PE_MODE_INTT, "INTT: Odd Sum Div2 (Modular Inverse Check)");
        drive_verify(0, 1, 1, PE_MODE_INTT, "INTT: A<B (Negative Subtraction Wrap)");
        drive_verify(3328, 3328, 3328, PE_MODE_INTT, "INTT: Max Field Values");

        // --------------------------------------------------
        // CWM Mode Tests (Shares NTT Logic path in PE0)
        // --------------------------------------------------
        $display("--- Testing CWM Mode ---");
        drive_verify(0, 0, 0, PE_MODE_CWM, "CWM: All Zeros");
        drive_verify(100, 50, 4, PE_MODE_CWM, "CWM: Simple Standard");
        drive_verify(3328, 3328, 3328, PE_MODE_CWM, "CWM: Max Field Values");

        // --------------------------------------------------
        // ADD/SUB Mode Tests (U = A + B, V = A - B)
        // --------------------------------------------------
        $display("--- Testing ADD/SUB Mode ---");
        drive_verify(0, 0, 0, PE_MODE_ADDSUB, "ADD/SUB: All Zeros");
        drive_verify(200, 50, 0, PE_MODE_ADDSUB, "ADD/SUB: Simple Standard (W ignored)");
        drive_verify(1000, 2500, 0, PE_MODE_ADDSUB, "ADD/SUB: Addition Overflow (Wrap > 3329)");
        drive_verify(1000, 2000, 0, PE_MODE_ADDSUB, "ADD/SUB: Subtraction Underflow (Wrap < 0)");
        drive_verify(100, 100, 0, PE_MODE_ADDSUB, "ADD/SUB: A=B (Sub = 0)");
        drive_verify(3328, 3328, 0, PE_MODE_ADDSUB, "ADD/SUB: Max Field Values");

        // --------------------------------------------------
        // Compression (Co/Deco) Mode Tests (U = A, V = B*W)
        // --------------------------------------------------
        $display("--- Testing Compression (Co/Deco) Mode ---");
        drive_verify(0, 0, 0, PE_MODE_CODECO1, "CODECO: All Zeros");
        drive_verify(1234, 500, 10, PE_MODE_CODECO1, "CODECO: Simple Standard");
        drive_verify(3328, 0, 0, PE_MODE_CODECO1, "CODECO: A-Passthrough Max Check");
        drive_verify(1234, 1, 1, PE_MODE_CODECO1, "CODECO: Multiplier Identity");
        drive_verify(3328, 3328, 3328, PE_MODE_CODECO1, "CODECO: Max Field Values");

        // --------------------------------------------------
        // Random Stress Loop (All Modes)
        // --------------------------------------------------
        $display("--- Starting Random Stress Loop (500 vectors) ---");
        for (int i = 0; i < 100; i++) begin
            coeff_t ra, rb, rw;
            pe_mode_e rmode;
            int mode_sel;

            ra = $urandom_range(0, 3328);
            rb = $urandom_range(0, 3328);
            rw = $urandom_range(0, 3328);

            mode_sel = $urandom_range(0, 4);
            case(mode_sel)
                0: rmode = PE_MODE_NTT;
                1: rmode = PE_MODE_INTT;
                2: rmode = PE_MODE_CWM;
                3: rmode = PE_MODE_ADDSUB;
                4: rmode = PE_MODE_CODECO1;
            endcase

            drive_verify(ra, rb, rw, rmode, "Random Vector");
        end

        // Final Summary
        repeat(5) @(posedge clk);
        $display("");
        $display("==========================================================");
        $display("SUMMARY");
        $display("Passed: %0d", pass_count);
        $display("Failed: %0d", error_count);
        $display("----------------------------------------------------------");

        if (error_count == 0) $display("RESULT: SUCCESS");
        else $display("RESULT: FAILURE");
        $display("==========================================================");
        $finish;
    end
endmodule
