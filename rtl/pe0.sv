/*
 * Module Name: pe0
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 *
 * Latency:
 */

module pe0 (
    input   logic           clk,
    input   logic           rst,

    // Input Operands (12-bit coefficients)
    input   coeff_t         a0_i,
    input   coeff_t         b0_i,
    input   coeff_t         w0_i,
    // Control Inputs (From the AU Controller)
    input   logic   [3:0]   ctrl_i,
    input   logic           valid_i,

    // Data Outputs
    output  coeff_t         u0_o,
    output  coeff_t         v0_o,
    output  logic           valid_o
);

    // =========================================================================
    // Logic Instantiations
    // =========================================================================

    // -------- Modular Adder Logic --------
    // Inputs
    logic   coeff_t mod_add_op1_i;
    logic   coeff_t mod_add_op2_i;
    logic           mod_add_valid_i;
    // Outputs
    logic   coeff_t mod_add_result_o;
    logic           mod_add_valid_o;

    // -------- Modular Subtractor Logic --------
    // Inputs
    logic   coeff_t mod_sub_op1_i;
    logic   coeff_t mod_sub_op2_i;
    logic           mod_sub_valid_i;
    // Outputs
    logic   coeff_t mod_sub_result_o;
    logic           mod_sub_valid_o;

    // -------- Modular Multiplier Logic --------
    // Inputs
    logic   coeff_t mod_mul_op1_i;
    logic   coeff_t mod_mul_op2_i;
    logic   coeff_t mod_mul_valid_i;
    // Outputs
    logic   coeff_t mod_mul_result_o;
    logic           mod_mul_valid_o;

    // -------- Modular Divider2 Logic --------
    logic   coeff_t mod_div_by_2_op_i;
    logic   coeff_t mod_div_by_2_op_o;

    // =========================================================================
    // Module Instantiations
    // =========================================================================

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .clk        (clk),
        .rst        (rst),

        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),
        .valid_i    (mod_add_valid_i),

        .result_o   (mod_add_result_o),
        .valid_o    (mod_add_valid_o)
    );
    assign mod_add_op1_i    =
    assign mod_add_op2_i    =
    assign mod_add_valid_i  =

    // -------- Modular Subtractor Instantiation --------
    mod_add u_mod_sub (
        .clk        (clk),
        .rst        (rst),

        .op1_i      (mod_sub_op1_i),
        .op2_i      (mod_sub_op2_i),
        .valid_i    (mod_sub_valid_i),

        .result_o   (mod_sub_result_o),
        .valid_o    (mod_sub_valid_o)
    );
    assign mod_sub_op1_i    =
    assign mod_sub_op2_i    =
    assign mod_sub_valid_i  =

    // -------- Modular Divider2 Instantiation --------
    mod_mul u_mod_mul (
        .clk        (clk),
        .rst        (rst),

        .op1_i      (mod_mul_op1_i),
        .op2_i      (mod_mul_op2_i),
        .valid_i    (mod_mul_valid_i),

        .result_o   (mod_mul_result_o),
        .valid_o    (mod_mul_valid_o)
    );
    assign mod_mul_op1_i    =
    assign mod_mul_op2_i    =
    assign mod_mul_valid_i  =

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o),
    );
    assign mod_div_by_2_op_i =
    assign mod_div_by_2_op_o =


endmodule
