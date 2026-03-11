/*
 * Module Name: cmi_addr_map
 * Author(s): Mai Komar
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," IEEE OJCAS, 2025. Section III-B4.
 *
 * Description:
 * Implements the conflict-free memory address mapping from the paper.
 * Takes a natural coefficient order index (0-255) and maps it to a
 * physical (bank_idx, bank_addr) pair using the paper's equations:
 *
 *   bankIdx   = Order >> λ          → order[7:2]  (which row,  0–63)
 *   bank_Addr = Order[λ-1:0]        → order[1:0]  (which bank, 0–3)
 *
 * Where λ=2, R=4 (4 banks, 4 PEs).
 *
 * Conflict-Free Guarantee:
 * Any group of 4 consecutive NTT indices always differ in their lower 2 bits,
 * guaranteeing all 4 PEs always land on 4 distinct banks every cycle.
 *
 * Latency: 0 Clock Cycles (Purely Combinational)
 */

import poly_arith_pkg::*;

module cmi_addr_map (
    // Natural order index of the coefficient (0 to N-1 = 255)
    input  logic [7:0]  order_i,

    // Which of the 4 BRAM banks this coefficient lives in (0–3)
    output logic [1:0]  bank_idx_o,

    // Row address within that bank (0–63)
    output logic [5:0]  bank_addr_o
);

    // Direct bit-slice — the entire mapping is just wires.
    // No logic gates on the critical path.
    //
    // order[1:0] → lower 2 bits  → selects which bank (0,1,2,3)
    // order[7:2] → upper 6 bits  → selects which row inside that bank
    assign bank_idx_o  = order_i[1:0];
    assign bank_addr_o = order_i[7:2];

endmodule