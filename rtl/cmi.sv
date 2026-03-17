/*
 * Module Name: cmi (Conflict-Free Memory Interface)
 * Author(s): Mai Komar
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on "UniPAM" from:
 * H. Jung et al., "Highly-Efficient Hardware Architecture for ML-KEM PQC
 * Standard," IEEE Open Journal of Circuits and Systems, 2025.
 *
 * Description:
 * Sits between the Arithmetic Unit (AU) and poly_mem_wrapper_4bank.
 * Translates the 4 coefficient indices from rd_wr_addr_gen into correctly
 * routed memory reads/writes, and delivers coefficients to the AU in the
 * correct lane order.
 *
 * Three responsibilities:
 *
 *  1. READ ROUTING
 *     Passes coeff_idx_i directly to poly_mem_wrapper as rd_idx_i.
 *     The wrapper's internal bankIdx/bankAddr functions handle the actual
 *     bank selection and address computation.
 *
 *  2. DATA CROSSBAR (2-cycle read latency correction)
 *     poly_ram_bank has 1-cycle synchronous read latency. The wrapper
 *     adds a second register stage on the output (rd_data_o). This means
 *     the wrapper's rd_data_o uses the CURRENT cycle's bankIdx to mux
 *     output lanes, but the data in those lanes is from 2 cycles ago.
 *
 *     The CMI corrects this by:
 *       - Delaying coeff_idx by 2 cycles  → coeff_idx_d2
 *       - Computing b_old[j] = bankIdx(coeff_idx_d2[j])
 *       - Computing b_now[k] = bankIdx(coeff_idx_i[k])
 *       - For each output lane j, finding k where b_now[k] == b_old[j],
 *         then routing mem_rd_data_i[k] → coeff_o[j]
 *     This recovers the correct coefficient for each AU lane regardless
 *     of how coeff_idx changes cycle-to-cycle.
 *
 *     NOTE: This crossbar works correctly only if all 4 banks are accessed
 *     every cycle (b_now is a permutation of {0,1,2,3}). rd_wr_addr_gen
 *     guarantees this by inserting dummy indices during R2 passes.
 *     However, during the 2-cycle pipeline fill at the start of each pass,
 *     coeff_idx_d2 holds stale/zero data — the AU should not consume
 *     coeff_o during these first 2 cycles. valid_o from rd_wr_addr_gen
 *     should be used to gate AU consumption with a 2-cycle delay.
 *
 *  3. WRITE-BACK
 *     After the AU finishes computing butterfly results (AU_PIPE cycles
 *     after the read), it asserts wr_en_i and drives wr_data_i.
 *     The CMI delays coeff_idx by AU_PIPE cycles (via delay_n) to produce
 *     the matching wr_idx, and passes it to the wrapper's write port.
 *
 *     IMPORTANT: If AU_PIPE < 2, the read and write indices to the same
 *     bank may collide in the same cycle. Verify using the wrapper's
 *     ready_o signal; stall the AU if ready_o is deasserted.
 *
 * Parameters:
 *   N         - Polynomial size (default 256, must be divisible by 4)
 *   W         - Coefficient storage width in bits (default 16)
 *   NUM_POLYS - Number of polynomials in memory (default 4)
 *   AU_PIPE   - AU pipeline depth in clock cycles (default 3).
 *               Must match the actual latency between read issuance and
 *               the AU asserting wr_en_i / wr_data_i.
 *
 * Latency:
 *   Read  → coeff_o valid: 2 clock cycles after rd_en_i asserted
 *   Write → committed to BRAM: 1 clock cycle after wr_en_i asserted
 */

import poly_arith_pkg::*;

module cmi #(
    parameter int N         = 256,
    parameter int W         = 16,
    parameter int NUM_POLYS = 4,
    parameter int AU_PIPE   = 3     // AU pipeline stages (read→writeback)
)(
    input  logic clk,
    input  logic rst,               // Active-high synchronous reset

    // ---- From rd_wr_addr_gen ----
    input  logic [3:0][7:0]  coeff_idx_i,   // 4 coefficient indices this cycle
    input  logic [3:0]       coeff_valid_i,  // Lane validity (4'b0011 for R2)

    // ---- AU control interface ----
    input  logic [$clog2(NUM_POLYS)-1:0] poly_id_i,  // Which polynomial
    input  logic                          v_i,         // Valid cycle
    input  logic                          rd_en_i,     // Issue a read this cycle
    input  logic [3:0]                    wr_en_i,     // Per-lane write enable (from AU)
    input  logic [3:0][W-1:0]             wr_data_i,   // AU butterfly results

    // ---- Coefficient output to AU ----
    output logic [3:0][W-1:0]             coeff_o,     // Lane-correct coefficients

    // ---- Status ----
    output logic                          ready_o,     // Forwarded from wrapper

    // ---- poly_mem_wrapper_4bank interface ----
    output logic [$clog2(NUM_POLYS)-1:0]  mem_poly_id_o,
    output logic                          mem_v_o,
    output logic                          mem_rd_en_o,
    output logic [3:0]                    mem_wr_en_o,
    output logic [3:0][7:0]               mem_rd_idx_o,  // 8-bit: log2(256)=8
    output logic [3:0][7:0]               mem_wr_idx_o,
    output logic [3:0][W-1:0]             mem_wr_data_o,
    input  logic [3:0][W-1:0]             mem_rd_data_i,
    input  logic                          mem_ready_i
);

    // =========================================================================
    // bankIdx function
    // Must match cmi_bank_idx inside poly_mem_wrapper_4bank exactly.
    // Computes bank = (sum of 2-bit chunks of index) mod 4.
    // This provides conflict-free access for any butterfly stride that is
    // a power-of-2 >= 2 (proven in paper Figure 6 / Section IV-B).
    // =========================================================================
    function automatic [1:0] bankIdx(input logic [7:0] idx);
        logic [3:0] s;
        s = idx[1:0] + idx[3:2] + idx[5:4] + idx[7:6];
        return s[1:0];
    endfunction

    // =========================================================================
    // 1. READ ROUTING: pass indices directly to wrapper
    // =========================================================================
    assign mem_poly_id_o = poly_id_i;
    assign mem_v_o       = v_i;
    assign mem_rd_en_o   = rd_en_i;

    generate
        for (genvar i = 0; i < 4; i++) begin : G_RD_IDX
            assign mem_rd_idx_o[i] = coeff_idx_i[i];
        end
    endgenerate

    // =========================================================================
    // 2. DATA CROSSBAR
    //
    // poly_ram_bank is synchronous (1-cycle read latency).
    // poly_mem_wrapper adds a second output register (1 more cycle).
    // Total latency: 2 cycles from issuing rd_idx to rd_data_o being valid.
    //
    // poly_mem_wrapper's output mux uses the CURRENT cycle's bankIdx:
    //   mem_rd_data_i[k] = data from bank(bankIdx(coeff_idx_NOW[k]))
    //                      at the address issued 2 cycles ago.
    //
    // The CMI crossbar undoes this by:
    //   - Tracking which bank each NOW lane corresponds to (b_now[k])
    //   - Tracking which bank each OLD lane needed       (b_old[j])
    //   - Routing: coeff_o[j] = mem_rd_data_i[k] where b_now[k] == b_old[j]
    //
    // coeff_idx_d1, coeff_idx_d2 are the indices from 1 and 2 cycles ago.
    // =========================================================================
    logic [3:0][7:0] coeff_idx_d1, coeff_idx_d2;

    always_ff @(posedge clk) begin
        if (rst) begin
            coeff_idx_d1 <= '0;
            coeff_idx_d2 <= '0;
        end else begin
            coeff_idx_d1 <= coeff_idx_i;
            coeff_idx_d2 <= coeff_idx_d1;
        end
    end

    // Bank indices: current cycle and 2 cycles ago
    logic [3:0][1:0] b_now;   // bankIdx for indices being presented NOW
    logic [3:0][1:0] b_old;   // bankIdx for indices whose data is NOW available

    always_comb begin
        for (int i = 0; i < 4; i++) begin
            b_now[i] = bankIdx(coeff_idx_i[i]);    // drives wrapper output mux
            b_old[i] = bankIdx(coeff_idx_d2[i]);   // matches data now in rd_data
        end
    end

    // Crossbar: for each desired output lane j (= "old" lane), scan current
    // lanes k to find which one the wrapper placed the data on.
    // Since conflict-free access guarantees b_now is a permutation of {0,1,2,3},
    // exactly one k satisfies b_now[k] == b_old[j] for each j.
    always_comb begin
        coeff_o = '0;
        for (int j = 0; j < 4; j++) begin
            for (int k = 0; k < 4; k++) begin
                if (b_now[k] == b_old[j])
                    coeff_o[j] = mem_rd_data_i[k];
            end
        end
    end

    // =========================================================================
    // 3. WRITE-BACK
    //
    // The AU reads coefficients at cycle N, computes the butterfly result
    // over AU_PIPE cycles, then asserts wr_en_i / wr_data_i at cycle N+AU_PIPE.
    // The write index must be the coefficient index from cycle N — i.e.,
    // coeff_idx_i delayed by AU_PIPE cycles.
    //
    // delay_n is instantiated per lane. Write enables are further gated by
    // coeff_valid_i so that dummy R2 lanes are never written back.
    // =========================================================================
    logic [3:0][7:0] wr_idx_delayed;
    logic [3:0]      coeff_valid_delayed;

    generate
        for (genvar i = 0; i < 4; i++) begin : G_WR_DELAY
            // Delay the coefficient index to align with AU write-back
            delay_n #(
                .DWIDTH (8),
                .DEPTH  (AU_PIPE)
            ) u_idx_delay (
                .clk    (clk),
                .rst    (rst),
                .data_i (coeff_idx_i[i]),
                .data_o (wr_idx_delayed[i])
            );

            // Delay lane validity alongside the index
            delay_n #(
                .DWIDTH (1),
                .DEPTH  (AU_PIPE)
            ) u_valid_delay (
                .clk    (clk),
                .rst    (rst),
                .data_i (coeff_valid_i[i]),
                .data_o (coeff_valid_delayed[i])
            );
        end
    endgenerate

    // Gate AU write enables with the delayed lane validity
    // (prevents dummy R2 indices being written with garbage AU data)
    assign mem_wr_en_o   = wr_en_i & coeff_valid_delayed;
    assign mem_wr_data_o = wr_data_i;

    generate
        for (genvar i = 0; i < 4; i++) begin : G_WR_IDX
            assign mem_wr_idx_o[i] = wr_idx_delayed[i];
        end
    endgenerate

    // =========================================================================
    // Pass-through ready signal
    // =========================================================================
    assign ready_o = mem_ready_i;

endmodule