/*
 * Module Name: modular_reduce
 * Target Standard: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Montgomery Reduction on a 32-bit signed integer.
 * This module implements the operation: Result = Z * R^-1 mod Q
 * * Parameters:
 * - Q = 3329
 * - R = 2^16
 * - Q_INV_NEG = -Q^-1 mod R = 3327
 *
 * Usage Notes:
 * 1. Input Domain: The input 'z_i' is usually the product of two numbers,
 * one of which must be in the "Montgomery Domain" (pre-multiplied by R).
 * 2. Output Range: This module performs "Lazy Reduction". The output 'res_o'
 * is in the range (-Q, Q), not strictly [0, Q). This is mathematically
 * valid for ML-KEM intermediate steps (like NTT/INTT) and saves logic area.
 * 3. Signedness: Uses signed arithmetic to support Centered Binomial
 * Distribution (negative coefficients).
 */

import poly_arith_pkg::*;

module modular_reduce(
    input   logic signed [31:0] z_i,    // 32-bit signed input value
    output  logic signed [15:0] res_o   // 16-bit signed result
);
    logic signed [15:0] m;
    logic signed [31:0] m_times_q;
    logic signed [31:0] t_wide;
    logic signed [15:0] t;

    // -------------------------------------------------------------------------
    // Step 1: Calculate the Reduction Factor (m)
    // Formula: m = (z_i * Q_INV_NEG) mod R
    // -------------------------------------------------------------------------
    // Note: z_i[15:0] is a part-select and technically unsigned, but for
    // modulo 2^16 arithmetic, the bit pattern for signed/unsigned multiplication
    // is identical. We cast to 16' to force truncation.
    assign m = 16'(z_i[15:0] * 16'(Q_INV_NEG));

    // -------------------------------------------------------------------------
    // Step 2: Perform the Reduction
    // Formula: t = (z_i + m * Q) / R
    // -------------------------------------------------------------------------
    // Since 'm' and 'Q' are both interpreted as signed 16-bit values,
    // this multiplication produces a signed 32-bit result.
    assign m_times_q = m * 16'(Q);
    assign t_wide    = z_i + m_times_q;

    // -------------------------------------------------------------------------
    // Step 3: Divide by R (Bit Shift)
    // -------------------------------------------------------------------------
    // By definition of Montgomery reduction, the lower 16 bits of 't_wide' are now 0
    assign t = t_wide[31:16];

    // -------------------------------------------------------------------------
    // Step 4: Output Assignment (Lazy Reduction)
    // -------------------------------------------------------------------------
    // The result 't' is mathematically guaranteed to be < 2*Q.
    // We do NOT perform the final conditional subtraction (if t > Q then t-Q)
    // because standard FIPS 203 hardware pipelines allow intermediate values
    // to float in the (-Q, Q) range for efficiency.
    assign res_o = t;

endmodule
