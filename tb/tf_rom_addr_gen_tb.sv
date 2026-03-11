// ==========================================================
// Testbench for Twiddle Factor ROM + Address Generator
// Author: Kiet Le
// Description: Verifies that the tf_rom and tf_addr_gen modules
//              produce the correct twiddle factor values for
//              all NTT and INTT passes, matching the mathematical
//              specification from FIPS 203 and the Inha University
//              Mixed-Radix-4/2 scheduling algorithm.
// ==========================================================
`timescale 1ns/1ps

module tf_rom_addr_gen_tb;

    import poly_arith_pkg::*;

    // =========================================================================
    // DUT Signals
    // =========================================================================
    logic           clk;
    logic           rst;

    // Address Generator
    logic           start;
    pe_mode_e       mode;

    logic [6:0]     addr0, addr1, addr2;
    logic           ag_valid, ag_busy;
    logic           is_intt;
    logic [1:0]     pass_num;

    // ROM
    coeff_t         w0, w1, w2, w3;

    // =========================================================================
    // Clock Generation (100 MHz)
    // =========================================================================
    localparam CLK_PERIOD = 10;

    initial clk = 0;
    always #(CLK_PERIOD / 2) clk = ~clk;

    // =========================================================================
    // DUT Instantiation
    // =========================================================================
    tf_addr_gen u_tf_addr_gen (
        .clk        (clk),
        .rst        (rst),
        .start_i    (start),
        .mode_i     (mode),
        .addr0_o    (addr0),
        .addr1_o    (addr1),
        .addr2_o    (addr2),
        .valid_o    (ag_valid),
        .busy_o     (ag_busy),
        .is_intt_o  (is_intt),
        .pass_o     (pass_num)
    );

    tf_rom u_tf_rom (
        .clk        (clk),
        .rst        (rst),
        .is_intt_i  (is_intt),
        .addr0_i    (addr0),
        .addr1_i    (addr1),
        .addr2_i    (addr2),
        .w0_o       (w0),
        .w1_o       (w1),
        .w2_o       (w2),
        .w3_o       (w3)
    );

    // =========================================================================
    // Helper: Bit Reversal
    // =========================================================================
    function automatic logic [6:0] bit_rev7 (input logic [6:0] val);
        bit_rev7 = {val[0], val[1], val[2], val[3], val[4], val[5], val[6]};
    endfunction

    // =========================================================================
    // Test Variables
    // =========================================================================
    int errors = 0;
    int cycle_count = 0;
    int total_tests = 0;

    // =========================================================================
    // Task: Wait for N clock cycles
    // =========================================================================
    task automatic wait_cycles(int n);
        repeat(n) @(posedge clk);
    endtask

    // =========================================================================
    // Task: Verify NTT Twiddle Factors
    // =========================================================================
    task automatic verify_ntt();
        int expected_addr0, expected_addr1, expected_addr2;
        int i_A, i_B_top, i_B_bot;
        int stg_a_base;
        int num_blocks, bfs_per_block;

        $display("===================================================");
        $display("  VERIFYING NTT TWIDDLE FACTOR GENERATION");
        $display("===================================================");

        // Start NTT
        @(posedge clk);
        mode = PE_MODE_NTT;
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;

        // Wait 1 cycle for addr_gen to start producing addresses
        // ROM has 1 cycle read latency, so we check ROM output 1 cycle after addr
        // But addr_gen is combinational, so addresses appear immediately on next cycle

        // === NTT Pass 1 (R4, stages 1&2): 1 block × 64 BFs ===
        $display("\n--- NTT Pass 1 (Radix-4, Stages 1&2) ---");
        stg_a_base = 1;
        num_blocks = 1;
        bfs_per_block = 64;

        for (int b = 0; b < num_blocks; b++) begin
            i_A     = stg_a_base + b;
            i_B_top = 2 * stg_a_base + 2 * b;
            i_B_bot = 2 * stg_a_base + 2 * b + 1;

            expected_addr0 = int'(bit_rev7(7'(i_B_top)));
            expected_addr1 = int'(bit_rev7(7'(i_A)));
            expected_addr2 = int'(bit_rev7(7'(i_B_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("NTT P1 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("NTT P1 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("NTT P1 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 1: Verified %0d cycles", num_blocks * bfs_per_block);

        // === NTT Pass 2 (R4, stages 3&4): 4 blocks × 16 BFs ===
        $display("\n--- NTT Pass 2 (Radix-4, Stages 3&4) ---");
        stg_a_base = 4;
        num_blocks = 4;
        bfs_per_block = 16;

        for (int b = 0; b < num_blocks; b++) begin
            i_A     = stg_a_base + b;
            i_B_top = 2 * stg_a_base + 2 * b;
            i_B_bot = 2 * stg_a_base + 2 * b + 1;

            expected_addr0 = int'(bit_rev7(7'(i_B_top)));
            expected_addr1 = int'(bit_rev7(7'(i_A)));
            expected_addr2 = int'(bit_rev7(7'(i_B_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("NTT P2 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("NTT P2 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("NTT P2 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 2: Verified %0d cycles", num_blocks * bfs_per_block);

        // === NTT Pass 3 (R4, stages 5&6): 16 blocks × 4 BFs ===
        $display("\n--- NTT Pass 3 (Radix-4, Stages 5&6) ---");
        stg_a_base = 16;
        num_blocks = 16;
        bfs_per_block = 4;

        for (int b = 0; b < num_blocks; b++) begin
            i_A     = stg_a_base + b;
            i_B_top = 2 * stg_a_base + 2 * b;
            i_B_bot = 2 * stg_a_base + 2 * b + 1;

            expected_addr0 = int'(bit_rev7(7'(i_B_top)));
            expected_addr1 = int'(bit_rev7(7'(i_A)));
            expected_addr2 = int'(bit_rev7(7'(i_B_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("NTT P3 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("NTT P3 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("NTT P3 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 3: Verified %0d cycles", num_blocks * bfs_per_block);

        // === NTT Pass 4 (R2, stage 7): 64 blocks × 2 BFs ===
        $display("\n--- NTT Pass 4 (Radix-2, Stage 7) ---");
        num_blocks = 64;
        bfs_per_block = 2;

        for (int b = 0; b < num_blocks; b++) begin
            expected_addr0 = int'(bit_rev7(7'(64 + b)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("NTT P4 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                // addr1 and addr2 should be 0 in Radix-2
                if (addr1 !== 7'd0) begin
                    $error("NTT P4 B%0d BF%0d: addr1 should be 0, got %0d",
                           b, j, addr1);
                    errors++;
                end
                if (addr2 !== 7'd0) begin
                    $error("NTT P4 B%0d BF%0d: addr2 should be 0, got %0d",
                           b, j, addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 4: Verified %0d cycles", num_blocks * bfs_per_block);

        // Wait for FSM to return to IDLE
        @(posedge clk);
        if (ag_busy) begin
            $error("NTT: Address generator still busy after all passes!");
            errors++;
        end

        $display("\n  NTT VERIFICATION COMPLETE: %0d errors in %0d tests",
                 errors, total_tests);
    endtask

    // =========================================================================
    // Task: Verify INTT Twiddle Factors
    // =========================================================================
    task automatic verify_intt();
        int expected_addr0, expected_addr1, expected_addr2;
        int i_A_top, i_A_bot, i_B;
        int highest_i_A, highest_i_B;
        int num_blocks, bfs_per_block;
        int intt_errors_start;

        intt_errors_start = errors;

        $display("\n===================================================");
        $display("  VERIFYING INTT TWIDDLE FACTOR GENERATION");
        $display("===================================================");

        // Start INTT
        @(posedge clk);
        mode = PE_MODE_INTT;
        start = 1'b1;
        @(posedge clk);
        start = 1'b0;

        // === INTT Pass 1 (R2, stage 1): 64 blocks × 2 BFs ===
        $display("\n--- INTT Pass 1 (Radix-2, Stage 1) ---");
        num_blocks = 64;
        bfs_per_block = 2;

        for (int b = 0; b < num_blocks; b++) begin
            expected_addr0 = int'(bit_rev7(7'(127 - b)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("INTT P1 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
            end
        end
        $display("  Pass 1: Verified %0d cycles", num_blocks * bfs_per_block);

        // === INTT Pass 2 (R4, stages 2&3): 16 blocks × 4 BFs ===
        $display("\n--- INTT Pass 2 (Radix-4, Stages 2&3) ---");
        num_blocks = 16;
        bfs_per_block = 4;
        // stage_A = 2, stage_B = 3 in INTT counting
        // highest_i_A = 2^(7-2+1) - 1 = 63
        // highest_i_B = 2^(7-3+1) - 1 = 31
        highest_i_A = 63;
        highest_i_B = 31;

        for (int b = 0; b < num_blocks; b++) begin
            i_A_top = highest_i_A - (2 * b);
            i_A_bot = highest_i_A - (2 * b) - 1;
            i_B     = highest_i_B - b;

            // INTT Radix-4 mapping:
            // w0 (PE0) = BitRev7(i_B)        -> Stage B twiddle
            // w1 (PE2 M1) = BitRev7(i_A_top) -> Stage A top twiddle
            // w2 (PE2 M2) = BitRev7(i_A_bot) -> Stage A bot twiddle
            expected_addr0 = int'(bit_rev7(7'(i_B)));
            expected_addr1 = int'(bit_rev7(7'(i_A_top)));
            expected_addr2 = int'(bit_rev7(7'(i_A_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("INTT P2 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("INTT P2 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("INTT P2 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 2: Verified %0d cycles", num_blocks * bfs_per_block);

        // === INTT Pass 3 (R4, stages 4&5): 4 blocks × 16 BFs ===
        $display("\n--- INTT Pass 3 (Radix-4, Stages 4&5) ---");
        num_blocks = 4;
        bfs_per_block = 16;
        // highest_i_A = 2^(7-4+1) - 1 = 15
        // highest_i_B = 2^(7-5+1) - 1 = 7
        highest_i_A = 15;
        highest_i_B = 7;

        for (int b = 0; b < num_blocks; b++) begin
            i_A_top = highest_i_A - (2 * b);
            i_A_bot = highest_i_A - (2 * b) - 1;
            i_B     = highest_i_B - b;

            expected_addr0 = int'(bit_rev7(7'(i_B)));
            expected_addr1 = int'(bit_rev7(7'(i_A_top)));
            expected_addr2 = int'(bit_rev7(7'(i_A_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("INTT P3 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("INTT P3 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("INTT P3 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 3: Verified %0d cycles", num_blocks * bfs_per_block);

        // === INTT Pass 4 (R4, stages 6&7): 1 block × 64 BFs ===
        $display("\n--- INTT Pass 4 (Radix-4, Stages 6&7) ---");
        num_blocks = 1;
        bfs_per_block = 64;
        // highest_i_A = 2^(7-6+1) - 1 = 3
        // highest_i_B = 2^(7-7+1) - 1 = 1
        highest_i_A = 3;
        highest_i_B = 1;

        for (int b = 0; b < num_blocks; b++) begin
            i_A_top = highest_i_A - (2 * b);
            i_A_bot = highest_i_A - (2 * b) - 1;
            i_B     = highest_i_B - b;

            expected_addr0 = int'(bit_rev7(7'(i_B)));
            expected_addr1 = int'(bit_rev7(7'(i_A_top)));
            expected_addr2 = int'(bit_rev7(7'(i_A_bot)));

            for (int j = 0; j < bfs_per_block; j++) begin
                @(posedge clk);
                total_tests++;

                if (addr0 !== 7'(expected_addr0)) begin
                    $error("INTT P4 B%0d BF%0d: addr0 mismatch. Got %0d, expected %0d",
                           b, j, addr0, expected_addr0);
                    errors++;
                end
                if (addr1 !== 7'(expected_addr1)) begin
                    $error("INTT P4 B%0d BF%0d: addr1 mismatch. Got %0d, expected %0d",
                           b, j, addr1, expected_addr1);
                    errors++;
                end
                if (addr2 !== 7'(expected_addr2)) begin
                    $error("INTT P4 B%0d BF%0d: addr2 mismatch. Got %0d, expected %0d",
                           b, j, addr2, expected_addr2);
                    errors++;
                end
            end
        end
        $display("  Pass 4: Verified %0d cycles", num_blocks * bfs_per_block);

        // Wait for FSM to return to IDLE
        @(posedge clk);
        if (ag_busy) begin
            $error("INTT: Address generator still busy after all passes!");
            errors++;
        end

        $display("\n  INTT VERIFICATION COMPLETE: %0d errors",
                 errors - intt_errors_start);
    endtask

    // =========================================================================
    // Task: Verify ROM Content Against Package
    // =========================================================================
    task automatic verify_rom_content();
        int rom_errors = 0;

        $display("\n===================================================");
        $display("  VERIFYING ROM CONTENT (NTT Forward Values)");
        $display("===================================================");

        // Manually check a few key values through the ROM
        // We'll use addr0 to read through all 128 entries
        for (int i = 0; i < 128; i++) begin
            @(posedge clk);
            // Force address (we're not using addr_gen here, just direct ROM check)
            force u_tf_rom.addr0_i = 7'(i);
            force u_tf_rom.is_intt_i = 1'b0;
            @(posedge clk);
            @(posedge clk); // Wait for registered output
            if (w0 !== ZETA_NTT_TABLE[i]) begin
                $error("ROM[%0d]: Got %0d, expected %0d", i, w0, ZETA_NTT_TABLE[i]);
                rom_errors++;
            end
        end
        release u_tf_rom.addr0_i;
        release u_tf_rom.is_intt_i;

        $display("  ROM Content Check: %0d errors in 128 entries", rom_errors);
        errors += rom_errors;
    endtask

    // =========================================================================
    // Task: Verify INTT Negation
    // =========================================================================
    task automatic verify_intt_negation();
        int neg_errors = 0;
        coeff_t expected_val;

        $display("\n===================================================");
        $display("  VERIFYING INTT NEGATION (Q - zeta)");
        $display("===================================================");

        for (int i = 1; i < 128; i++) begin // Skip i=0 (zeta^0 = 1, -1 mod Q = 3328)
            @(posedge clk);
            force u_tf_rom.addr0_i = 7'(i);
            force u_tf_rom.is_intt_i = 1'b1;
            @(posedge clk);
            @(posedge clk);
            expected_val = 12'(Q) - ZETA_NTT_TABLE[i];
            if (w0 !== expected_val) begin
                $error("INTT ROM[%0d]: Got %0d, expected %0d (Q - %0d)",
                       i, w0, expected_val, ZETA_NTT_TABLE[i]);
                neg_errors++;
            end
        end
        release u_tf_rom.addr0_i;
        release u_tf_rom.is_intt_i;

        $display("  INTT Negation Check: %0d errors in 127 entries", neg_errors);
        errors += neg_errors;
    endtask

    // =========================================================================
    // Task: Verify omega_4 Output
    // =========================================================================
    task automatic verify_omega4();
        $display("\n===================================================");
        $display("  VERIFYING OMEGA_4 OUTPUT (PE3 Twiddle Factor)");
        $display("===================================================");

        // NTT mode
        force u_tf_rom.is_intt_i = 1'b0;
        @(posedge clk);
        @(posedge clk);
        if (w3 !== 12'd1729) begin
            $error("omega_4 NTT: Got %0d, expected 1729", w3);
            errors++;
        end else begin
            $display("  omega_4 NTT = %0d (PASS)", w3);
        end

        // INTT mode
        force u_tf_rom.is_intt_i = 1'b1;
        @(posedge clk);
        @(posedge clk);
        if (w3 !== 12'd1600) begin
            $error("omega_4 INTT: Got %0d, expected 1600", w3);
            errors++;
        end else begin
            $display("  omega_4 INTT = %0d (PASS)", w3);
        end

        release u_tf_rom.is_intt_i;
    endtask

    // =========================================================================
    // Main Test Sequence
    // =========================================================================
    initial begin
        $display("===================================================");
        $display("  TF_ROM + TF_ADDR_GEN Testbench");
        $display("===================================================\n");

        // Initialize
        rst   = 1'b1;
        start = 1'b0;
        mode  = PE_MODE_NTT;

        // Reset sequence
        repeat(5) @(posedge clk);
        rst = 1'b0;
        repeat(2) @(posedge clk);

        // ---- Test 1: ROM Content ----
        verify_rom_content();

        // ---- Test 2: INTT Negation ----
        verify_intt_negation();

        // ---- Test 3: Omega_4 ----
        verify_omega4();

        // Allow some settling time
        repeat(5) @(posedge clk);

        // ---- Test 4: Full NTT Address Sequence ----
        verify_ntt();

        // Allow settling between tests
        repeat(10) @(posedge clk);

        // ---- Test 5: Full INTT Address Sequence ----
        verify_intt();

        // =========================================================================
        // Final Report
        // =========================================================================
        repeat(5) @(posedge clk);

        $display("\n===================================================");
        if (errors == 0) begin
            $display("  ALL TESTS PASSED (%0d total checks)", total_tests);
        end else begin
            $display("  FAILED: %0d errors in %0d total checks", errors, total_tests);
        end
        $display("===================================================\n");

        $finish;
    end

    // =========================================================================
    // Timeout Watchdog
    // =========================================================================
    initial begin
        #1_000_000; // 1 ms timeout
        $error("TESTBENCH TIMEOUT!");
        $finish;
    end

endmodule
