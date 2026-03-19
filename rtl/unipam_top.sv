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
 */

import poly_arith_pkg::*;

module unipam_top (
    input  logic       clk,
    input  logic       rst,

    // ---- Control Interface (From Main System) ----
    // TODO: start_i will eventually route to the controller. Placeholder for TF testing.
    input  logic       start_i,
    input  pe_mode_e   op_type_i

    // output logic            ready_o,
    // output logic            done_o,

    // // ---- Memory Interface (To SRAM / CMI) ----
    // output logic            mem_read_en_o,
    // output logic            mem_write_en_o,
    // output logic [7:0]      rAddr_o,
    // output logic [7:0]      wAddr_o,

    // // Input Data Bus (From SRAM)
    // input  coeff_t          a0_i, b0_i,
    // input  coeff_t          a1_i, b1_i, c0_i, c1_i,
    // input  coeff_t          a2_i, b2_i,
    // input  coeff_t          a3_i, b3_i, w3_i,

    // // Output Data Bus (To SRAM)
    // output coeff_t          u0_o, v0_o,
    // output coeff_t          u1_o, v1_o,
    // output coeff_t          u2_o, v2_o, m_o,
    // output coeff_t          u3_o, v3_o
);

    // ==========================================
    // Internal Signals
    // ==========================================

    // Controller Signals
    logic            tf_start;
    logic [1:0]      pass_idx;
    logic [3:0][7:0] coeff_idx;

    // Address Generator & Twiddle Factor Signals
    logic [5:0]      tf_addr;
    logic            is_radix2;
    logic [1:0]      pass_out;
    logic            is_intt;

    coeff_t          w0;       // PE0 twiddle factor
    coeff_t          w1;       // PE2 multiplier 1 twiddle factor
    coeff_t          w2;       // PE2 multiplier 2 twiddle factor
    coeff_t          w3;       // PE3 Radix-4 root (omega_4)

    // ==========================================
    // Operational Logic
    // ==========================================

    // Decode INTT mode based on operation type
    always_comb begin
        case (op_type_i)
            PE_MODE_NTT  : is_intt = 1'b0;
            PE_MODE_INTT : is_intt = 1'b1;
            default      : is_intt = 1'b0; // Safe default
        endcase
    end

    // ==========================================
    // Sub-Module Instantiations
    // ==========================================

    logic cmi_v;
    logic cmi_rd_en;
    // ---- Controller ----
    unipam_controller u_controller (
        .clk                (clk),
        .rst                (rst),
        .start_i            (start_i),
        .op_type_i          (op_type_i),
        .poly_id_i          (),
        .ready_o            (),
        .done_o             (),
        .tf_start_o         (tf_start),
        .pass_idx_o         (pass_idx),
        .pe_ctrl_o          (),
        .pe_valid_o         (),
        .cmi_ready_i        (1'b1),        // Placeholder for now
        .cmi_v_o            (cmi_v),
        .cmi_rd_en_o        (cmi_rd_en),
        .cmi_poly_id_o      (),
        .cmi_coeff_idx_o    (coeff_idx),
        .cmi_coeff_valid_o  (),
        .block_cnt_o        (),
        .bf_cnt_o           (),
        .cmi_wb_latency_o   ()
    );

    cmi cmi (
        .clk                (clk),
        .rst                (rst),
        .coeff_idx_i        (coeff_idx),
        .coeff_valid_i      (),
        .poly_id_i          (),
        .v_i                (),
        .rd_en_i            (),
        .wr_en_i            (),
        .wr_data_i          (),
        .coeff_o            (),
        .ready_o            (),
        .mem_poly_id_o      (),
        .mem_v_o            (),
        .mem_rd_en_o        (),
        .mem_wr_en_o        (),
        .mem_rd_idx_o       (),
        .mem_wr_idx_o       (),
        .mem_wr_data_o      (),
        .mem_rd_data_i      (),
        .mem_ready_i        ()
    );

    // ---- Twiddle Factor Address Generator ----
    tf_addr_gen u_tf_addr_gen (
        .clk                (clk),
        .rst                (rst),
        .start_i            (tf_start),    // Triggered by controller
        .ctrl_i             (op_type_i),
        .pass_idx_i         (pass_idx),    // From controller
        .tf_addr_o          (tf_addr),     // To ROM
        .is_radix2_o        (is_radix2),   // To ROM
        .valid_o            (),
        .pass_o             (pass_out)
    );

    // ---- Twiddle Factor ROM ----
    tf_rom u_tf_rom (
        .clk                (clk),
        .rst                (rst),
        .is_intt_i          (is_intt),     // Decoded from op_type_i
        .is_radix2_i        (is_radix2),   // From Address Gen
        .tf_addr_i          (tf_addr),     // From Address Gen
        .w0_o               (w0),          // To PE Unit
        .w1_o               (w1),          // To PE Unit
        .w2_o               (w2),          // To PE Unit
        .w3_o               (w3)           // To PE Unit
    );

    // ---- Processing Element (PE) Unit ----
    // Awaiting CMI integration; twiddle factors and operands will be hardcoded/routed later.
    pe_unit u_pe_unit (
        .clk                (clk), // Added missing clk/rst connections
        .rst                (rst),
        .valid_i            (),
        .ctrl_i             (),
        .mode_i             (),
        .op_a0_i            (),
        .op_a1_i            (),
        .op_a2_i            (),
        .op_a3_i            (),
        .op_b0_i            (),
        .op_b1_i            (),
        .op_b2_i            (),
        .op_b3_i            (),
        .z0_o               (),
        .z1_o               (),
        .z2_o               (),
        .z3_o               (),
        .valid_o            ()
    );

endmodule