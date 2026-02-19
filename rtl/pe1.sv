/*
 * Module Name: pe1 (Processing Element 1)
 * Author(s): Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 */

import poly_arith_pkg::*;

module pe0 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a1_i,
    input   coeff_t         b1_i,
    input   coeff_t         c0_i,
    input   coeff_t         c1_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u1_o,
    output  coeff_t         v1_o,
    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================


    // ============= Delay Register Wires =============

    // IDK HOW MANY PROPAGATION CYCLES I NEED TBH
    // -------- Delay 4 Valid Propagation Register --------
    logic   delay_4_valid_data_i;
    logic   delay_4_valid_data_o;

    // -------- Delay 3 Valid Propagation Register --------
    logic   delay_3_valid_data_i;
    logic   delay_3_valid_data_o;


    // -------- Delay 1 Valid Propagation Register --------
    logic   delay_1_valid_data_i;
    logic   delay_1_valid_data_o;

    // -------- Delay 1 Addition Output Register --------
    coeff_t delay_1_add_data_i;
    coeff_t delay_1_add_data_o;

    // -------- Delay 1 Unified Add/Sub Output Register --------
    coeff_t delay_1_uni_add_sub_data_i;
    coeff_t delay_1_uni_add_sub_data_o;

    // -------- Delay 3 U1 Input Register Logic --------
    coeff_t delay_3_u1_data_i;
    coeff_t delay_3_u1_data_o;

    // -------- Delay 3 V1 Input Register Logic --------
    coeff_t delay_3_v1_data_i;
    coeff_t delay_3_v1_data_o;

    // ============= Arithmetic Module Wires =============
    // -------- Modular Adder Logic --------
    // Inputs
    coeff_t mod_add_op1_i;
    coeff_t mod_add_op2_i;
    // Outputs
    coeff_t mod_add_result_o;

    // -------- Modular Divider2 Logic --------
    coeff_t mod_div_by_2_op_i;
    coeff_t mod_div_by_2_op_o;

    // -------- Modular Uni Add/Sub Logic --------
    coeff_t mod_uni_add_sub_op1_i;
    coeff_t mod_uni_add_sub_op2_i;
    wire mod_uni_add_sub_is_sub_i;
    // Outputs
    coeff_t mod_uni_add_sub_result_o;

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================

    // -------- Delay 4 Valid Propagation Register --------
    // For NTT, INTT, and CWM Modes (4-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (4)
    ) u_delay_4_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_4_valid_data_i),
        .data_o(delay_4_valid_data_o)
    );
    // Gate input: Only enter pipe during 4-cycle modes
    assign delay_4_valid_data_i = (ctrl_i == PE_MODE_NTT || ctrl_i == PE_MODE_INTT || ctrl_i == PE_MODE_CWM) ? valid_i : 1'b0;

    // -------- Delay 3 U1 Input Register --------
    // For Co/Deco Modes (3-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (3)
    ) u_delay_3_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_u1_data_i),
        .data_o(delay_3_u1_data_o)
    );
    // Gate input: Only enter pipe during 3-cycle modes
    // -------- Delay 3 U1 Input Register Logic --------
    assign delay_3_u1_data_i = ctrl_i[2] ? mod_div_by_2_op_o : delay_1_add_data_o;

    // -------- Delay 3 V1 Input Register --------
    // For Co/Deco Modes (3-cycle latency)
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (3)
    ) u_delay_3_valid (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_v1_data_i),
        .data_o(delay_3_v1_data_o)
    );
    //
    assign delay_3_v1_data_i = delay_1_uni_add_sub_data_o;
    // Gate input: Only enter pipe during 3-cycle modes
    // -------- Delay 3 V1 Input Register Logic --------

    // -------- Delay 1 Addition Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_add (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_add_data_i),
        .data_o(delay_1_add_data_o)
    );
    assign delay_1_add_data_i = mod_add_result_o;

    // -------- Delay 1 Unified Add/Sub Output Register --------
    mod_uni_add_sub #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_uni_add_sub (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_uni_add_sub_data_i),
        .data_o(delay_1_uni_add_sub_data_o)
    );
    assign delay_1_uni_add_sub_data_i = mod_uni_add_sub_result_o;

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = a1_i;
    assign mod_add_op2_i    = ctrl_i[1] ? b1_i : c1_i;

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o)
    );
    assign mod_div_by_2_op_i = delay_1_add_data_o;

// -------- Modular Divider2 Instantiation --------
    mod_uni_add_sub u_uni_add_sub(
        // Inputs: Two 12-bit coefficients (0 to 3328)
        .op1_i              (mod_uni_add_sub_op1_i),
        .op2_i              (mod_uni_add_sub_op2_i),
        .is_sub_i           (mod_uni_add_sub_is_sub_i),
        .result_o           (mod_uni_add_sub_result_o)
    );

    assign mod_uni_add_sub_op1_i = b1_i;
    assign mod_uni_add_sub_op2_i = ctrl_i[1] ? a1_i : c0_i;
    assign mod_uni_add_sub_is_sub_i = ctrl_i[1];

    // =========================================================================
    // PE Outputs
    // =========================================================================

    assign u1_o     = ctrl_i[3] ? delay_3_u1_data_o : delay_1_add_data_o;
    assign v1_o     = ctrl_i[3] ? delay_3_v1_data_o : delay_1_uni_add_sub_data_i;

    // COME BACK TO VALID PROPAGATION LOGIC
endmodule
