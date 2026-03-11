// ==========================================================
// Testbench for CMI Address Mapper (Combinational)
// Author(s): [Your Name]
// Target: FIPS 203 (ML-KEM) - Conflict-Free Memory Interface
// DUT: cmi_addr_map
//
// Test Suites:
//   1. Known-Value Spot Checks  — hand-calculated boundary values
//   2. Bit-Slice Correctness    — all 256 indices verified against golden model
//   3. Conflict-Free Guarantee  — all 64 butterfly groups checked
//   4. Output Range Check       — no X/Z, no out-of-range outputs
// ==========================================================

`timescale 1ns/1ps

module cmi_addr_map_tb();

    import poly_arith_pkg::*;

    // ------------------------------------------------------
    // Signals
    // ------------------------------------------------------
    logic clk;
    logic [7:0] order_i;
    logic [1:0] bank_idx_o;
    logic [5:0] bank_addr_o;

    // ------------------------------------------------------
    // Verification Stats
    // ------------------------------------------------------
    int pass_count = 0;
    int fail_count = 0;

    // Struct to hold expected values for checker
    typedef struct {
        logic [7:0] order;
        logic [1:0] exp_bank_idx;
        logic [5:0] exp_bank_addr;
        string      name;
    } transaction_t;

    transaction_t expected_q[$];

    // ------------------------------------------------------
    // Clock Generation (100 MHz)
    // ------------------------------------------------------
    initial clk = 0;
    always #5 clk = ~clk;

    // ------------------------------------------------------
    // DUT Instantiation (Combinational — no clk/rst needed)
    // ------------------------------------------------------
    cmi_addr_map dut (
        .order_i    (order_i),
        .bank_idx_o (bank_idx_o),
        .bank_addr_o(bank_addr_o)
    );

    // ------------------------------------------------------
    // Golden Model
    // Direct translation of paper equations (lambda=2, R=4):
    //   bank_idx  = order[1:0]
    //   bank_addr = order[7:2]
    // ------------------------------------------------------
    function automatic logic [1:0] golden_bank_idx(input logic [7:0] order);
        return order[1:0];
    endfunction

    function automatic logic [5:0] golden_bank_addr(input logic [7:0] order);
        return order[7:2];
    endfunction

    // ------------------------------------------------------
    // Task: Drive one coefficient index and queue expectation
    // ------------------------------------------------------
    task automatic send(
        input logic [7:0] order,
        input string      name
    );
        transaction_t trans;

        @(posedge clk);
        order_i <= order;

        trans.order         = order;
        trans.exp_bank_idx  = golden_bank_idx(order);
        trans.exp_bank_addr = golden_bank_addr(order);
        trans.name          = name;

        expected_q.push_back(trans);
    endtask

    // ------------------------------------------------------
    // Monitor / Checker (At Negedge — same cycle, 0 latency)
    // ------------------------------------------------------
    always @(negedge clk) begin
        if (expected_q.size() > 0) begin
            transaction_t trans;
            trans = expected_q.pop_front();

            if (bank_idx_o === trans.exp_bank_idx &&
                bank_addr_o === trans.exp_bank_addr) begin
                pass_count++;
            end else begin
                $display("[FAIL] %s | order=%0d (8'b%08b)",
                    trans.name, trans.order, trans.order);
                if (bank_idx_o !== trans.exp_bank_idx)
                    $display("       bank_idx  -- Expected: %0d | Got: %0d",
                        trans.exp_bank_idx, bank_idx_o);
                if (bank_addr_o !== trans.exp_bank_addr)
                    $display("       bank_addr -- Expected: %0d | Got: %0d",
                        trans.exp_bank_addr, bank_addr_o);
                fail_count++;
            end
        end
    end

    // ==========================================================
    // Main Stimulus
    // ==========================================================
    initial begin
        // Initialize
        order_i = '0;

        $display("==================================================");
        $display("Starting cmi_addr_map Verification (Combinational)");
        $display("Paper equations: bank_idx=order[1:0], bank_addr=order[7:2]");
        $display("==================================================");

        repeat(5) @(posedge clk);

        // --------------------------------------------------
        // TEST 1 — Known-Value Spot Checks
        // Hand-calculated from paper equations.
        // order binary -> bank_idx (bits[1:0]) | bank_addr (bits[7:2])
        //
        //  order=0   -> 00000000 -> bank=0, addr=0
        //  order=1   -> 00000001 -> bank=1, addr=0
        //  order=2   -> 00000010 -> bank=2, addr=0
        //  order=3   -> 00000011 -> bank=3, addr=0
        //  order=4   -> 00000100 -> bank=0, addr=1
        //  order=7   -> 00000111 -> bank=3, addr=1
        //  order=252 -> 11111100 -> bank=0, addr=63
        //  order=255 -> 11111111 -> bank=3, addr=63
        // --------------------------------------------------
        $display("--- TEST 1: Known-Value Spot Checks ---");
        send(8'd0,   "Spot: order=0   (min, bank=0, addr=0)");
        send(8'd1,   "Spot: order=1   (bank=1, addr=0)");
        send(8'd2,   "Spot: order=2   (bank=2, addr=0)");
        send(8'd3,   "Spot: order=3   (bank=3, addr=0)");
        send(8'd4,   "Spot: order=4   (bank=0, addr=1)");
        send(8'd7,   "Spot: order=7   (bank=3, addr=1)");
        send(8'd252, "Spot: order=252 (bank=0, addr=63)");
        send(8'd255, "Spot: order=255 (max, bank=3, addr=63)");

        wait(expected_q.size() == 0);
        repeat(2) @(posedge clk);

        // --------------------------------------------------
        // TEST 2 — Bit-Slice Correctness (all 256 indices)
        // Every index applied and checked against golden model.
        // --------------------------------------------------
        $display("--- TEST 2: Bit-Slice Correctness (all 256 indices) ---");
        for (int i = 0; i < 256; i++) begin
            send(8'(i), $sformatf("BitSlice: order=%0d", i));
        end

        wait(expected_q.size() == 0);
        repeat(2) @(posedge clk);

        // --------------------------------------------------
        // TEST 3 — Conflict-Free Guarantee
        // For all 64 butterfly groups (base, base+1, base+2, base+3),
        // sample each lane sequentially (DUT is combinational) and
        // verify all 4 bank_idx values are unique.
        //
        // A conflict means 2+ PEs would try to access the same bank
        // in the same cycle — the CMI must never allow this.
        // --------------------------------------------------
        $display("--- TEST 3: Conflict-Free Guarantee (64 groups) ---");
        begin
            logic [1:0] bidx  [4];
            logic [5:0] baddr [4];
            logic conflict;
            int cf_fail;
            cf_fail = 0;

            for (int base = 0; base < 256; base += 4) begin
                // Sample all 4 lanes (combinational — order doesn't matter)
                for (int lane = 0; lane < 4; lane++) begin
                    @(posedge clk);
                    order_i = 8'(base + lane);
                    #1; // combinational settle
                    bidx [lane] = bank_idx_o;
                    baddr[lane] = bank_addr_o;
                end

                // Check all 6 unique pairs for bank_idx collisions
                conflict = (bidx[0]==bidx[1]) || (bidx[0]==bidx[2]) ||
                           (bidx[0]==bidx[3]) || (bidx[1]==bidx[2]) ||
                           (bidx[1]==bidx[3]) || (bidx[2]==bidx[3]);

                if (conflict) begin
                    $display("[FAIL] Conflict-Free: base=%0d banks=[%0d %0d %0d %0d]",
                        base, bidx[0], bidx[1], bidx[2], bidx[3]);
                    fail_count++;
                    cf_fail++;
                end else begin
                    pass_count++;
                end
            end

            if (cf_fail == 0)
                $display("       All 64 groups are conflict-free.");
        end

        repeat(2) @(posedge clk);

        // --------------------------------------------------
        // TEST 4 — Output Range and X/Z Check (all 256 indices)
        // bank_idx must stay in [0,3], bank_addr in [0,63].
        // No X or Z allowed on any output for any valid input.
        // --------------------------------------------------
        $display("--- TEST 4: Output Range and X/Z Check (all 256 indices) ---");
        for (int i = 0; i < 256; i++) begin
            @(posedge clk);
            order_i = 8'(i);
            #1; // combinational settle

            if ($isunknown(bank_idx_o) || $isunknown(bank_addr_o)) begin
                $display("[FAIL] Range/X: order=%0d -- X or Z detected on output", i);
                fail_count++;
            end else if (bank_idx_o > 2'd3) begin
                $display("[FAIL] Range: order=%0d -- bank_idx=%0d out of range [0,3]",
                    i, bank_idx_o);
                fail_count++;
            end else if (bank_addr_o > 6'd63) begin
                $display("[FAIL] Range: order=%0d -- bank_addr=%0d out of range [0,63]",
                    i, bank_addr_o);
                fail_count++;
            end else begin
                pass_count++;
            end
        end

        repeat(2) @(posedge clk);

        // --------------------------------------------------
        // Final Report
        // --------------------------------------------------
        $display("\n--- FINAL REPORT ---");
        $display("Tests Passed: %0d", pass_count);
        $display("Tests Failed: %0d", fail_count);

        if (fail_count == 0 && pass_count > 0)
            $display("SIMULATION RESULT: SUCCESS\n");
        else begin
            $display("SIMULATION RESULT: FAILURE\n");
            $fatal(1, "cmi_addr_map_tb: Testbench failed.");
        end

        $finish;
    end

endmodule