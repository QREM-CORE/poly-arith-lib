/*
 * Module Name: modular_reduce
 * Author: Kiet Le
 * Target Standard: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs high-speed modular reduction (Z mod 3329) for 24-bit unsigned integers.
 * This module is optimized for the ML-KEM polynomial arithmetic context, specifically
 * reducing the 24-bit product of two 12-bit coefficients.
 *
 * Based on the method described in:
 * H. Jung, Q. Dang Truong and H. Lee,
 * "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"
 * in IEEE Open Journal of Circuits and Systems, vol. 6, pp. 356-369, 2025
 *
 * Algorithm: Table-Based Modular Reduction (Radix-16 Decomposition)
 * Instead of expensive division or DSP-based Montgomery reduction, this design:
 * 1. Decomposes the 24-bit input into three 4-bit upper chunks and one 12-bit lower chunk.
 * 2. Uses Look-Up Tables (LUTs) to retrieve pre-computed residues for weighted positions.
 * 3. Sums the partial residues and the lower chunk.
 *
 * Architecture: 2-Stage Fixed-Latency Pipeline
 * - Stage 1 (Cycle 0): Parallel LUT lookup and summation of partial results.
 * - Stage 2 (Cycle 1): Final correction using parallel subtraction to ensure strict output range [0, 3328].
 *
 * Key Features:
 * - DSP-Free: Uses only logic (LUTs) and adders, saving DSP slices for multipliers.
 * - Constant Time: Fixed 2-cycle latency suitable for high-frequency pipelining.
 * - Area Efficient: Replaces large dividers with compact MUX-based logic.
 */

import poly_arith_pkg::*;

module modular_reduce(
    input   logic           clk,
    input   logic           rst,

    input   logic           valid_i,
    input   logic [23:0]    product_i,    // 24-bit input value

    output  logic           valid_o,
    output  logic [11:0]    result_o   // 12-bit result
);

    // ============================================================
    // STAGE 1: LUT Lookup & Summation (Cycle 0 -> Cycle 1)
    // ============================================================

    // 1. Slice Inputs
    logic [3:0]  chunk_23_20, chunk_19_16, chunk_15_12;
    logic [11:0] chunk_11_00;

    assign chunk_23_20 = product_i[23:20];
    assign chunk_19_16 = product_i[19:16];
    assign chunk_15_12 = product_i[15:12];
    assign chunk_11_00 = product_i[11:0];

    // 2. LUT Logic (Combinational) - "Precomputed" values
    logic [11:0] lut_val_15_12, lut_val_19_16, lut_val_23_20;

    always_comb begin
        // LUT [15:12] (Weight: 767)
        case(chunk_15_12)
            4'd0:       lut_val_15_12 = 12'd0;
            4'd1:       lut_val_15_12 = 12'd767;
            4'd2:       lut_val_15_12 = 12'd1534;
            4'd3:       lut_val_15_12 = 12'd2301;
            4'd4:       lut_val_15_12 = 12'd3068;
            4'd5:       lut_val_15_12 = 12'd506;
            4'd6:       lut_val_15_12 = 12'd1273;
            4'd7:       lut_val_15_12 = 12'd2040;
            4'd8:       lut_val_15_12 = 12'd2807;
            4'd9:       lut_val_15_12 = 12'd245;
            4'd10:      lut_val_15_12 = 12'd1012;
            4'd11:      lut_val_15_12 = 12'd1779;
            4'd12:      lut_val_15_12 = 12'd2546;
            4'd13:      lut_val_15_12 = 12'd3313;
            4'd14:      lut_val_15_12 = 12'd751;
            4'd15:      lut_val_15_12 = 12'd1518;
            default:    lut_val_15_12 = '0;
        endcase

        // LUT [19:16] (Weight: 2285)
        case(chunk_19_16)
            4'd0:       lut_val_19_16 = 12'd0;
            4'd1:       lut_val_19_16 = 12'd2285;
            4'd2:       lut_val_19_16 = 12'd1241;
            4'd3:       lut_val_19_16 = 12'd197;
            4'd4:       lut_val_19_16 = 12'd2482;
            4'd5:       lut_val_19_16 = 12'd1438;
            4'd6:       lut_val_19_16 = 12'd394;
            4'd7:       lut_val_19_16 = 12'd2679;
            4'd8:       lut_val_19_16 = 12'd1635;
            4'd9:       lut_val_19_16 = 12'd591;
            4'd10:      lut_val_19_16 = 12'd2876;
            4'd11:      lut_val_19_16 = 12'd1832;
            4'd12:      lut_val_19_16 = 12'd788;
            4'd13:      lut_val_19_16 = 12'd3073;
            4'd14:      lut_val_19_16 = 12'd2029;
            4'd15:      lut_val_19_16 = 12'd985;
            default:    lut_val_19_16 = '0;
        endcase

        // LUT [23:20] (Weight: 3270)
        case(chunk_23_20)
            4'd0:       lut_val_23_20 = 12'd0;
            4'd1:       lut_val_23_20 = 12'd3270;
            4'd2:       lut_val_23_20 = 12'd3211;
            4'd3:       lut_val_23_20 = 12'd3152;
            4'd4:       lut_val_23_20 = 12'd3093;
            4'd5:       lut_val_23_20 = 12'd3034;
            4'd6:       lut_val_23_20 = 12'd2975;
            4'd7:       lut_val_23_20 = 12'd2916;
            4'd8:       lut_val_23_20 = 12'd2857;
            4'd9:       lut_val_23_20 = 12'd2798;
            4'd10:      lut_val_23_20 = 12'd2739;
            default:    lut_val_23_20 = '0;
        endcase
    end

    // 3. Summation
    logic [13:0] stage1_sum;
    assign stage1_sum = lut_val_23_20 + lut_val_19_16 + lut_val_15_12 + chunk_11_00;

    // ============================================================
    // PIPELINE REGISTER (The "Barrier" between cycles)
    // ============================================================
    logic [13:0] sum_reg;
    logic        valid_pipe;

    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            sum_reg    <= '0;
            valid_pipe <= 1'b0;
        end else begin
            sum_reg    <= stage1_sum;
            valid_pipe <= valid_i; // Pass the valid flag forward
        end
    end

    // ============================================================
    // STAGE 2: Final Correction (Cycle 1 -> Cycle 2)
    // ============================================================

    // Parallel Subtraction (Optimization for speed)
    logic [13:0] sub_1q, sub_2q, sub_3q, sub_4q;
    assign sub_1q = sum_reg - 14'd3329;
    assign sub_2q = sum_reg - 14'd6658;
    assign sub_3q = sum_reg - 14'd9987;
    assign sub_4q = sum_reg - 14'd13316;

    always_comb begin
        if      (sum_reg >= 14'd13316) result_o = sub_4q[11:0];
        else if (sum_reg >= 14'd9987)  result_o = sub_3q[11:0];
        else if (sum_reg >= 14'd6658)  result_o = sub_2q[11:0];
        else if (sum_reg >= 14'd3329)  result_o = sub_1q[11:0];
        else                           result_o = sum_reg[11:0];
    end

    // Pass valid signal to output
    assign valid_o = valid_pipe;

endmodule
