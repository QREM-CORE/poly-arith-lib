/*
 * Module Name: pe0
 * Author(s): Kiet Le
 * Target: FIPS 203 (ML-KEM / Kyber)
 *
 * Description:
 *
 * Latency:
 */

import poly_arith_pkg::*;

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
    // ============= Input Registers =============
    logic   coeff_t a0, b0, w0;
    logic   [3:0]   ctrl;

    // ============= Delay Register Wires =============
    // -------- Delay 4 Valid Propagation Register --------
    logic   coeff_t delay_4_valid_data_i;
    logic   coeff_t delay_4_valid_data_o;

    // -------- Delay 1 W0 Input Register Logic --------
    logic   coeff_t delay_1_w0_data_i;
    logic   coeff_t delay_1_w0_data_o;

    // -------- Delay 3 Register --------
    logic   coeff_t delay_3_data_i;
    logic   coeff_t delay_3_data_o;

    // -------- Delay 1 Addition Output Register --------
    logic   coeff_t delay_1_add_data_i;
    logic   coeff_t delay_1_add_data_o;

    // -------- Delay 1 Subtraction Output Register --------
    logic   coeff_t delay_1_sub_data_i;
    logic   coeff_t delay_1_sub_data_o;

    // ============= Arithmetic Module Wires =============
    // -------- Modular Adder Logic --------
    // Inputs
    logic   coeff_t mod_add_op1_i;
    logic   coeff_t mod_add_op2_i;
    // Outputs
    logic   coeff_t mod_add_result_o;

    // -------- Modular Subtractor Logic --------
    // Inputs
    logic   coeff_t mod_sub_op1_i;
    logic   coeff_t mod_sub_op2_i;
    // Outputs
    logic   coeff_t mod_sub_result_o;

    // -------- Modular Multiplier Logic --------
    // Inputs
    logic   coeff_t mod_mul_op1_i;
    logic   coeff_t mod_mul_op2_i;
    // Outputs
    logic   coeff_t mod_mul_result_o;

    // -------- Modular Divider2 Logic --------
    logic   coeff_t mod_div_by_2_op_i;
    logic   coeff_t mod_div_by_2_op_o;

    // =========================================================================
    // Input Registration
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            a0      <= '0;
            b0      <= '0;
            w0      <= '0;
            ctrl    <= '0;
        end else begin
            if (valid_i) begin
                a0      <= a0_i;
                b0      <= b0_i;
                w0      <= w0_i;
                ctrl    <= ctrl_i;
            end
        end
    end

    // =========================================================================
    // Delay Register Instantiations
    // =========================================================================
    // -------- Delay 4 Valid Propagation Register --------
    delay_n #(
        .DWIDTH (1),
        .DEPTH  (4)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_4_valid_data_i),
        .data_o(delay_4_valid_data_o)
    );
    assign delay_4_valid_data_i = valid_i;

    // -------- W0 Input Delay 1 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_w0_data_i),
        .data_o(delay_1_w0_data_o)
    );
    assign delay_1_w0_data_i    = w0;

    // -------- Delay 3 Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (3)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_3_data_i),
        .data_o(delay_3_data_o)
    );
    assign delay_3_data_i = ctrl_i[0] ? mod_div_by_2_op_o : a0;

    // -------- Delay 1 Addition Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_add_data_i),
        .data_o(delay_1_add_data_o)
    );
    assign delay_1_add_data_o = 

    // -------- Delay 1 Subtraction Output Register --------
    delay_n #(
        .DWIDTH (COEFF_WIDTH), // 12-bit
        .DEPTH  (1)
    ) u_delay_1_w0 (
        .clk(clk),
        .rst(rst),

        .data_i(delay_1_sub_data_i),
        .data_o(delay_1_sub_data_o)
    );
    assign delay_1_sub_data_i = 

    // =========================================================================
    // Arithmetic Module Instantiations
    // =========================================================================

    // -------- Modular Adder Instantiation --------
    mod_add u_mod_add (
        .op1_i      (mod_add_op1_i),
        .op2_i      (mod_add_op2_i),

        .result_o   (mod_add_result_o)
    );
    assign mod_add_op1_i    = 
    assign mod_add_op2_i    =

    // -------- Modular Subtractor Instantiation --------
    mod_add u_mod_sub (
        .op1_i      (mod_sub_op1_i),
        .op2_i      (mod_sub_op2_i),

        .result_o   (mod_sub_result_o)
    );
    assign mod_sub_op1_i    =
    assign mod_sub_op2_i    =

    // -------- Modular Divider2 Instantiation --------
    mod_mul u_mod_mul (
        .clk        (clk),
        .rst        (rst),

        .op1_i      (mod_mul_op1_i),
        .op2_i      (mod_mul_op2_i),

        .result_o   (mod_mul_result_o)
    );
    assign mod_mul_op1_i    = ctrl_i[0] ? delay_1_w0_data_o : w0;
    assign mod_mul_op2_i    = ctrl_i[0] ? delay_1_sub_data_o : b0;

    // -------- Modular Divider2 Instantiation --------
    mod_div_by_2 u_mod_div_by_2 (
        .op_i       (mod_div_by_2_op_i),
        .op_o       (mod_div_by_2_op_o),
    );
    assign mod_div_by_2_op_i =
    assign mod_div_by_2_op_o =


endmodule
