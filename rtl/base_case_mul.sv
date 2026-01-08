/*
 * Module Name: base_case_mul
 * Author: Kiet Le
 * Target Standard: FIPS 203 (ML-KEM) - Algorithm 12
 *
 * Description:
 * Implements the "BaseCaseMultiply" operation for degree-one polynomials
 * in the NTT domain. This is the fundamental building block for the
 * pointwise multiplication of two NTT-transformed polynomials.
 *
 * Mathematical Operation:
 * Computes the product of two linear polynomials mod (X^2 - zeta):
 * a(x) = a0 + a1*X
 * b(x) = b0 + b1*X
 * c(x) = a(x) * b(x) mod (X^2 - zeta)
 *
 * Resulting Equations:
 * c0 = a0*b0 + a1*b1*zeta  (modulo q)
 * c1 = a0*b1 + a1*b0       (modulo q)
 *
 * Input Requirements:
 * - Inputs a0, a1, b0, b1 are 16-bit signed coefficients (coeff_t).
 * - Input 'zeta' MUST be the specific constant for this coefficient pair,
 *   fetched from the ROM and ZERO-EXTENDED to 16-bit signed.
 *
 * Implementation Details:
 * - Uses 16x16 signed multipliers (DSP inference).
 * - Implements "Lazy Reduction" internally by accumulating 32-bit products
 *   before performing the final Montgomery reduction.
 */

import poly_arith_pkg::*;

module base_case_mul (
    // Input Pair A
    input  coeff_t  a0, // The Even coefficient (e.g., index 0)
    input  coeff_t  a1, // The Odd coefficient  (e.g., index 1)

    // Input Pair B
    input  coeff_t  b0,
    input  coeff_t  b1,

    // The Zeta Constant for this specific pair (from ROM)
    input  coeff_t  zeta,

    // Output Pair
    output coeff_t  c0,
    output coeff_t  c1
);
    // ========= Computation of c0: a0*b0 + a1*b1*zeta =========
    logic signed [31:0] p0,         // a0 * b0
                        p1,         // a1 * b1
                        pzeta,      // reduce(a1 * b1) * zeta
                        c0_wide;    // a0*b0 + a1*b1*zeta
    coeff_t p1_reduced; // reduce(p1)

    assign p0       = a0 * b0;
    assign p1       = a1 * b1;
    assign pzeta    = p1_reduced * zeta;
    assign c0_wide  = p0 + pzeta;

    modular_reduce mod1 (
        .z_i    (p1),
        .res_o  (p1_reduced)
    );

    modular_reduce mod2 (
        .z_i    (c0_wide),
        .res_o  (c0)
    );

    // =========== Computation of c1: a0*b1 + a1*b0 ============
    logic signed [31:0] c1_wide;
    assign c1_wide  = a0*b1 + a1*b0;

    modular_reduce mod3 (
        .z_i    (c1_wide),
        .res_o  (c1)
    );

endmodule
