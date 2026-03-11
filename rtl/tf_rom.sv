/*
 * Module Name: tf_rom (Twiddle Factor ROM)
 * Author(s): Salwan Aldhahab
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Synchronous Read-Only Memory storing the 128 pre-computed twiddle factors
 * (zeta^BitRev7(i) mod Q) required for the NTT/INTT/CWM operations of ML-KEM.
 *
 * The ROM stores ONLY the forward NTT twiddle factors (unsigned, 12-bit). For
 * the Inverse NTT (INTT), the address generator provides the same indices and
 * this module applies a conditional negation: -zeta mod Q = Q - zeta. This
 * eliminates the need for a second ROM, halving the storage requirement.
 *
 * Architecture:
 * - 3 independent read ports (w0, w1, w2) to supply twiddle factors to PE0,
 *   PE2 multiplier 1, and PE2 multiplier 2 simultaneously in Radix-4 passes.
 * - A fixed output port for the Radix-4 root of unity (omega_4) used by PE3.
 * - All outputs are registered (1 clock cycle read latency).
 *
 * Port Mapping to pe_unit:
 * | ROM Port  | pe_unit Input | Radix-4 Role        | Radix-2 Role    |
 * |-----------|---------------|---------------------|-----------------|
 * | w0_o      | w0_i (PE0)    | Stage B top zeta    | Butterfly zeta  |
 * | w1_o      | w1_i (PE2 M1) | Stage A zeta        | Unused (0)      |
 * | w2_o      | w2_i (PE2 M2) | Stage B bottom zeta | Unused (0)      |
 * | w3_o      | w3_i (PE3 TF) | omega_4^1 / ^(-1)   | Unused (0)      |
 *
 * Memory Organization:
 * - 128 x 12-bit entries (192 bytes total)
 * - Indexed by 7-bit address [6:0]
 * - Content: ZETA_NTT_TABLE from poly_arith_pkg (zeta^BitRev7(i) mod 3329)
 */

import poly_arith_pkg::*;

module tf_rom (
    input   logic           clk,
    input   logic           rst,

    // ---- Control ----
    input   logic           is_intt_i,      // 1 = INTT mode (negate outputs)

    // ---- Read Port 0 (PE0 twiddle factor: w0) ----
    input   logic [6:0]     addr0_i,
    output  coeff_t         w0_o,

    // ---- Read Port 1 (PE2 multiplier 1 twiddle factor: w1) ----
    input   logic [6:0]     addr1_i,
    output  coeff_t         w1_o,

    // ---- Read Port 2 (PE2 multiplier 2 twiddle factor: w2) ----
    input   logic [6:0]     addr2_i,
    output  coeff_t         w2_o,

    // ---- Fixed Port 3 (PE3 Radix-4 root: omega_4) ----
    output  coeff_t         w3_o
);

    // =========================================================================
    // ROM Storage
    // =========================================================================
    // The ROM is initialized from the ZETA_NTT_TABLE defined in poly_arith_pkg.
    // Entry[i] = zeta^BitRev7(i) mod 3329, for i in [0, 127].
    // Entry[0] = 1 (zeta^0), which is the identity and typically not used.

    coeff_t rom [0:127];

    // Initialize ROM from package parameter
    initial begin
        for (int i = 0; i < 128; i++) begin
            rom[i] = ZETA_NTT_TABLE[i];
        end
    end

    // =========================================================================
    // Combinational ROM Read
    // =========================================================================
    coeff_t rom_data0, rom_data1, rom_data2;

    assign rom_data0 = rom[addr0_i];
    assign rom_data1 = rom[addr1_i];
    assign rom_data2 = rom[addr2_i];

    // =========================================================================
    // INTT Negation Logic
    // =========================================================================
    // For INTT: zeta^(-k) mod Q = Q - zeta^k mod Q (for non-zero values).
    // Since Q = 3329, negation is simply 3329 - value.
    // Special case: if value == 0 (shouldn't happen for valid indices), keep 0.

    coeff_t w0_ntt, w1_ntt, w2_ntt;

    assign w0_ntt = rom_data0;
    assign w1_ntt = rom_data1;
    assign w2_ntt = rom_data2;

    coeff_t w0_intt, w1_intt, w2_intt;

    assign w0_intt = 12'(Q) - rom_data0;
    assign w1_intt = 12'(Q) - rom_data1;
    assign w2_intt = 12'(Q) - rom_data2;

    // Select NTT or INTT output
    coeff_t w0_sel, w1_sel, w2_sel;

    assign w0_sel = is_intt_i ? w0_intt : w0_ntt;
    assign w1_sel = is_intt_i ? w1_intt : w1_ntt;
    assign w2_sel = is_intt_i ? w2_intt : w2_ntt;

    // =========================================================================
    // Output Registration (1 Clock Cycle Read Latency)
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            w0_o <= '0;
            w1_o <= '0;
            w2_o <= '0;
        end else begin
            w0_o <= w0_sel;
            w1_o <= w1_sel;
            w2_o <= w2_sel;
        end
    end

    // =========================================================================
    // Fixed Omega_4 Output (PE3 Twiddle Factor)
    // =========================================================================
    // omega_4 = zeta^(N/4) = zeta^64 = 17^64 mod 3329 = 1729  (NTT)
    // omega_4^(-1) = 1600                                       (INTT)
    // These are constant per transform direction and never change.

    localparam coeff_t OMEGA_4_NTT  = 12'd1729;
    localparam coeff_t OMEGA_4_INTT = 12'd1600;

    always_ff @(posedge clk) begin
        if (rst) begin
            w3_o <= '0;
        end else begin
            w3_o <= is_intt_i ? OMEGA_4_INTT : OMEGA_4_NTT;
        end
    end

endmodule
