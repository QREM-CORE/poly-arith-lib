/*
 * Module Name: modular_reduce
 * Author: Kiet Le
 * Target Standard: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 *
 */

import poly_arith_pkg::*;

module modular_reduce(
    input   logic           clk
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
    logic [3:0]     chunk_23_20, chunk_19_16, chunk_15_12;
    logic [11:0]    chunk_11_00;

    wire [3:0]  chunk_23_20 = product_i[23:20];
    wire [3:0]  chunk_19_16 = product_i[19:16];
    wire [3:0]  chunk_15_12 = product_i[15:12];
    wire [11:0] chunk_11_00  = product_i[11:0];



endmodule
