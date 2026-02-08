/*
 * Module Name: mod_uni_add_sub
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 * Performs Modular Addition or Subtraction: (A +/- B) mod 3329.
 * * Optimization:
 * Subtraction is implemented as Addition of the modular inverse:
 * (A - B) mod Q  ==  (A + (Q - B)) mod Q
 * This allows a single adder to handle both operations.
 *
 * Latency: 2 Clock Cycles
 * - Cycle 0: Input Registration & Operand Prep
 * - Cycle 1: Addition, Reduction, & Output Registration
 */

import poly_arith_pkg::*;

module mod_uni_add_sub(
    input   logic   clk,
    input   logic   rst,

    // Inputs: Two 12-bit coefficients (0 to 3328)
    input   coeff_t op1_i,
    input   coeff_t op2_i,
    input   logic   is_sub_i, // Control: 1 = Subtract, 0 = Add
    input   logic   valid_i,

    // Output: 12-bit result
    output  coeff_t result_o,
    output  logic   valid_o
);

    coeff_t         op1_reg, op2_final_reg;
    logic           valid_reg1;

    logic [12:0]    sum_raw;
    logic [12:0]    sum_reduced;
    coeff_t         final_result_wire;

    // =========================================================================
    // CYCLE 0: Input Registration & Subtraction Handling
    // =========================================================================

    always_ff @(posedge clk) begin
        if (rst) begin
            op1_reg       <= '0;
            op2_final_reg <= '0; // Will hold B (if add) or Q-B (if sub)
            valid_reg1    <= 1'b0;
        end else begin
            op1_reg    <= op1_i;
            valid_reg1 <= valid_i;

            // OPTIMIZATION: Handle Subtraction by negating 'B' immediately
            // If Subtracting: Use (Q - B).
            // If Adding:      Use B.
            // Note: If op2 is 0, (3329 - 0) = 3329. This is handled correctly by reduction later.
            if (is_sub_i) begin
                op2_final_reg <= 13'(Q) - op2_i;
            end else begin
                op2_final_reg <= op2_i;
            end
        end
    end

    // =========================================================================
    // COMBINATIONAL LOGIC: Addition & Reduction
    // =========================================================================

    // 1. Unified Addition
    // Max Value Add: 3328 + 3328 = 6656
    // Max Value Sub: 3328 + (3329 - 0) = 6657
    // Both fit in 13 bits.
    assign sum_raw = op1_reg + op2_final_reg;

    // 2. Reduction (Conditional Subtraction of Q)
    assign sum_reduced = sum_raw - 13'(Q);

    // If sum >= Q, we subtract Q. Otherwise pass sum.
    assign final_result_wire = (sum_raw >= 13'(Q)) ? sum_reduced[11:0] : sum_raw[11:0];

    // =========================================================================
    // CYCLE 1: Output Registration
    // =========================================================================

    always_ff @(posedge clk) begin
        if (rst) begin
            result_o <= '0;
            valid_o  <= 1'b0;
        end else begin
            result_o <= final_result_wire;
            valid_o  <= valid_reg1;
        end
    end

endmodule
