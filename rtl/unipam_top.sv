/*
 * Module Name: unipam_top
 * Author(s): Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Top module for the whole UNIPAM system.
 *
 */

/*
 * Module Name: unipam_top
 * Author(s): Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Top module for the whole UNIPAM system. Ties together the controller, 
 * twiddle factor address generator, ROM, and the Processing Element array.
 */

import poly_arith_pkg::*;

module unipam_top (
    input  logic             clk,
    input  logic             rst,

    // ---- Control Interface (From Main System) ----
    input  logic             start_i,
    input  pe_mode_e         op_type_i,

    output logic             ready_o,
    output logic             done_o,

    // ---- Memory Interface (To SRAM / CMI) ----
    output logic             mem_read_en_o,
    output logic             mem_write_en_o,
    output logic [7:0]       rAddr_o,
    output logic [7:0]       wAddr_o,

    // Input Data Bus (From SRAM)
    input  coeff_t           a0_i, b0_i,
    input  coeff_t           a1_i, b1_i, c0_i, c1_i,
    input  coeff_t           a2_i, b2_i,
    input  coeff_t           a3_i, b3_i, w3_i,

    // Output Data Bus (To SRAM)
    output coeff_t           u0_o, v0_o,
    output coeff_t           u1_o, v1_o,
    output coeff_t           u2_o, v2_o, m_o,
    output coeff_t           u3_o, v3_o
);

endmodule