/*
 * Module Name: pe2 (Processing Element 2)
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
    input   coeff_t         a2_i,
    input   coeff_t         b2_i,
    input   coeff_t         w1_i,
    input   coeff_t         w2_i,
    // Control Inputs (From the AU Controller)
    input   pe_mode_e       ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u2_o,
    output  coeff_t         v2_o,
    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // ============= Delay Register Wires =============

    // -------- W1 Input Delay 1 Register --------
    coeff_t delay_1_w1_data_i;
    coeff_t delay_1_w1_data_o;

    // -------- W2 Input Delay 1 Register --------
    coeff_t delay_2_w1_data_i;
    coeff_t delay_2_w1_data_o;

    // -------- Delay 1 Addition Output Register --------
    // Inputs
    coeff_t delay_1_add_data_i;
    // Outputs
    coeff_t delay_1_add_data_o;


    // -------- Delay 1 Subtraction Output Register --------
    // Inputs
    coeff_t delay_1_sub_data_i;
    // Outputs
    coeff_t delay_1_sub_data_o;

    // ============= Arithmetic Module Wires =============

    // -------- Modular Multiplier 1 Logic --------
    // Inputs
    coeff_t mod_mul_1_op1_i;
    coeff_t mod_mul_1_op2_i;
    // Outputs
    coeff_t mod_mul_1_result_o;

    // -------- Modular Multiplier 2 Logic --------
    coeff_t mod_mul_2_op1_i;
    coeff_t mod_mul_2_op2_i;
    // Outputs
    coeff_t mod_mul_2_result_o;

    // -------- Modular Adder Logic --------
    coeff_t mod_add_op1_i;
    coeff_t mod_add_op2_i;
    // Outputs
    coeff_t mod_add_result_o;

    // -------- Modular Subtractor Logic --------
    // Inputs
    coeff_t mod_sub_op1_i;
    coeff_t mod_sub_op2_i;
    // Outputs
    coeff_t mod_sub_result_o;

    // U2 input mux
    coeff_t u2_mux_i;
    coeff_t v2_mux_i;

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================

    // -------- W1 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w1 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w1_data_i),
        .data_o(delay_1_w1_data_o)
    );

    // -------- W2 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w1 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_2_w1_data_i),
        .data_o(delay_2_w1_data_o)
    );

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

    // -------- Delay 1 Subtraction Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_sub (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_sub_data_i),
        .data_o(delay_1_sub_data_o)
    );
    assign delay_1_sub_data_i = mod_sub_result_o;

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Multiplier 1 Instantiation --------
    mod_mul u_mod_mul_1 (
        .clk           (clk),
        .rst           (rst),
        .valid_i       (),

        .op1_i         (mod_mul_1_op1_i),
        .op2_i         (mod_mul_1_op2_i),

        .result_o      (mod_mul_1_result_o),
        .valid_o       ()
    );

    assign mod_mul_1_op1_i = ctrl_i[0] ? delay_1_add_data_o : a2_i;
    assign mod_mul_1_op2_i = ctrl_i[0] ? delay_1_w1_data_o : w1_i;

    // -------- Modular Multiplier 1 Instantiation --------
    mod_mul u_mod_mul_2 (
        .clk           (clk),
        .rst           (rst),
        .valid_i       (),

        .op1_i         (mod_mul_2_op1_i),
        .op2_i         (mod_mul_2_op2_i),

        .result_o      (mod_mul_2_result_o),
        .valid_o       ()
    );

    assign mod_mul_2_op1_i = ctrl_i[0] ? delay_2_w1_data_o : w2_i;
    assign mod_mul_2_op2_i = ctrl_i[0] ? delay_1_sub_data_o : b2_i;

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = ctrl_i[0] ? a2_i : mod_mul_1_result_o;
    assign mod_add_op2_i    = ctrl_i[0] ? b2_i : mod_mul_2_result_o;

    // -------- Modular Subtractor Instantiation --------
    mod_sub u_mod_sub (
        .op1_i      (mod_sub_op1_i),
        .op2_i      (mod_sub_op2_i),

        .result_o   (mod_sub_result_o)
    );
    assign mod_sub_op1_i    = ctrl_i[0] ? b2_i : mod_mul_2_result_o;
    assign mod_sub_op2_i    = ctrl_i[0] ? a2_i : mod_mul_1_result_o;

    // =========================================================================
    // PE Outputs
    // =========================================================================

    assign u2_mux_i = ctrl_i[2] ? mod_mul_1_result_o : delay_1_add_data_i;
    assign v2_mux_i = ctrl_i[2] ? mod_mul_2_result_o : delay_1_sub_data_o;

    assign u2_o = ctrl_i[1] ? u2_mux_i : mod_mul_1_result_o;
    assign v2_o = ctrl_i[1] ? v2_mux_i : mod_mul_2_result_o;

    // COME BACK TO VALID PROPAGATION LOGIC
endmodule
