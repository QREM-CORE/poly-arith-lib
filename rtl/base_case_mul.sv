/*
 * Module Name: base_case_mul
 * Author: Kiet Le
 * Target Standard: FIPS 203 (ML-KEM / Kyber) - Algorithm 12
 *
 * Description:
 * Implements the "BaseCaseMultiply" operation for degree-one polynomials
 * in the NTT domain using the Karatsuba multiplication technique.
 * This unit performs the pointwise multiplication of two vector elements
 * modulo (X^2 - zeta).
 *
 * Based on the method described in:
 * H. Jung, Q. Dang Truong and H. Lee,
 * "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"
 * in IEEE Open Journal of Circuits and Systems, vol. 6, pp. 356-369, 2025
 *
 * Mathematical Operation:
 * Given two inputs in the ring R_q[X]/(X^2 - zeta):
 * a(X) = a0 + a1*X
 * b(X) = b0 + b1*X
 *
 * The product c(X) = a(X) * b(X) is calculated as:
 * c0 = a0*b0 + a1*b1*zeta  (Note: X^2 becomes zeta)
 * c1 = a0*b1 + a1*b0
 *
 * Hardware Optimization (Karatsuba):
 * A naive implementation would require 5 multiplications (4 for coefficients + 1 for zeta).
 * This module uses Karatsuba to reduce the coefficient mixing steps, resulting in
 * a total of 4 multiplications:
 *
 * 1. Mult High:  P_high = a1 * b1
 * 2. Mult Low:   P_low  = a0 * b0
 * 3. Mult Sum:   P_sum  = (a0 + a1) * (b0 + b1)
 * 4. Mult Zeta:  P_zeta = P_high * zeta
 *
 * The final coefficients are derived as:
 * c0 = P_low + P_zeta
 * c1 = P_sum - P_high - P_low
 *
 * Architecture:
 * - 2-Cycle Fixed Latency Pipeline.
 * - Utilizes 3 parallel modular reducers in Step 1.
 * - Utilizes 1 final modular reducer in Step 2 for the c0 term.
 */

import poly_arith_pkg::*;

module base_case_mul (
    input           clk,
    input           rst,
    input           valid_i,

    // Input Pair A
    input  coeff_t  a0_i,
    input  coeff_t  a1_i,

    // Input Pair B
    input  coeff_t  b0_i,
    input  coeff_t  b1_i,

    // Zeta
    input  coeff_t  zeta_i,

    // Output Pair
    output coeff_t  c0_o,
    output coeff_t  c1_o,
    output          valid_o
);

    // ============================================================
    // STAGE 0: Raw Multiplication & Karatsuba Setup
    // ============================================================

    // 1. Raw Products (24-bit)
    logic [23:0] prod_high, prod_low;
    assign prod_high = a1_i * b1_i;
    assign prod_low  = a0_i * b0_i;

    // 2. Middle Term Calculation (Karatsuba)
    // Formula: (a0+a1)(b0+b1) - a1b1 - a0b0
    // Note: In standard integer math, this result is always positive.
    logic [12:0] sum_a, sum_b; // 13-bit to hold sum of two 12-bit coeffs
    assign sum_a = a0_i + a1_i;
    assign sum_b = b0_i + b1_i;

    logic [25:0] prod_sum;
    assign prod_sum = sum_a * sum_b;

    logic [25:0] mid_term_raw;
    assign mid_term_raw = prod_sum - prod_high - prod_low;

    // ============================================================
    // STEP 1: Parallel Reduction (Cycles 0 -> 1)
    // ============================================================

    // We reduce ALL three terms immediately to save register width later.
    // Latency of modular_reduce is 1 Cycle.
    // Outputs (red_high, etc.) are valid at T+1.

    logic val_high_valid, val_low_valid, val_mid_valid;
    logic [11:0] red_high, red_low, red_mid;

    // Reducer 1: High Term (a1 * b1)
    modular_reduce REDUCE_HIGH (
        .clk(clk), .rst(rst),
        .valid_i(valid_i),
        .product_i(prod_high),
        .valid_o(val_high_valid),
        .result_o(red_high)
    );

    // Reducer 2: Low Term (a0 * b0)
    modular_reduce REDUCE_LOW (
        .clk(clk), .rst(rst),
        .valid_i(valid_i),
        .product_i(prod_low),
        .valid_o(val_low_valid),
        .result_o(red_low)
    );

    // Reducer 3: Middle Term
    modular_reduce REDUCE_MID (
        .clk(clk), .rst(rst),
        .valid_i(valid_i),
        .product_i(mid_term_raw),
        .valid_o(val_mid_valid),
        .result_o(red_mid)
    );

    // ============================================================
    // STEP 2: Zeta Multiplication & C0 Calculation (Cycle 2)
    // ============================================================

    // 1. Zeta Pipeline (Must match REDUCE_HIGH latency = 1 cycle)
    logic [11:0] zeta_d1;
    always_ff @(posedge clk) begin
        if (rst) begin
            zeta_d1 <= '0;
        end else begin
            zeta_d1 <= zeta_i;
        end
    end

    // 2. Math for C0 (Combinational at T+1)
    logic [23:0] term_zeta;
    logic [23:0] sum_c0_raw;

    assign term_zeta  = red_high * zeta_d1;
    assign sum_c0_raw = term_zeta + red_low;

    // ============================================================
    // STEP 3: Final Reduction & Output Alignment (Cycles 1 -> 2)
    // ============================================================

    // 1. Final C0 Reducer (Takes 1 cycle. Total Latency = 1+1 = 2)
    logic val_c0_valid;
    logic [11:0] c0_final;

    modular_reduce REDUCE_FINAL_C0 (
        .clk(clk), .rst(rst),
        .valid_i(val_high_valid),
        .product_i(sum_c0_raw),
        .valid_o(val_c0_valid),
        .result_o(c0_final)
    );

    // 2. C1 Delay Line (Must align with C0 at T+2)
    // Middle term was ready at T+1. We need to delay it 1 cycle.
    logic [11:0] c1_d1;

    always_ff @(posedge clk) begin
        if (rst) c1_d1 <= '0;
        else     c1_d1 <= red_mid;
    end

    // ============================================================
    // OUTPUT ASSIGNMENT
    // ============================================================

    assign c0_o = c0_final;
    assign c1_o = c1_d1;
    assign valid_o = val_c0_valid;

endmodule
