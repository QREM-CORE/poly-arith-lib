// ==========================================================
// Testbench for Zeta Constants (Algorithm 11)
// Author: Kiet Le
// Description: Verifies that the hardware ROM (Unsigned) matches
//              the FIPS 203 Appendix A (Signed) specification.
// ==========================================================
`timescale 1ns/1ps

module zeta_mul_table_tb;

    // 1. Define the Constants (Reference from FIPS 203 / Your Image)
    // We use 'int' so we can store negative numbers easily for comparison
    int expected_signed [0:127] = '{
        17,   -17,   2761, -2761, 583,  -583,  2649, -2649,
        1637, -1637, 723,  -723,  2288, -2288, 1100, -1100,
        1409, -1409, 2662, -2662, 3281, -3281, 233,  -233,
        756,  -756,  2156, -2156, 3015, -3015, 3050, -3050,
        1703, -1703, 1651, -1651, 2789, -2789, 1789, -1789,
        1847, -1847, 952,  -952,  1461, -1461, 2687, -2687,
        939,  -939,  2308, -2308, 2437, -2437, 2388, -2388,
        733,  -733,  2337, -2337, 268,  -268,  641,  -641,
        1584, -1584, 2298, -2298, 2037, -2037, 3220, -3220,
        375,  -375,  2549, -2549, 2090, -2090, 1645, -1645,
        1063, -1063, 319,  -319,  2773, -2773, 757,  -757,
        2099, -2099, 561,  -561,  2466, -2466, 2594, -2594,
        2804, -2804, 1092, -1092, 403,  -403,  1026, -1026,
        1143, -1143, 2150, -2150, 2775, -2775, 886,  -886,
        1722, -1722, 1212, -1212, 1874, -1874, 1029, -1029,
        2110, -2110, 2935, -2935, 885,  -885,  2154, -2154
    };

    // 2. Import your Hardware ROM
    import poly_arith_pkg::*;

    int errors = 0;
    int q = 3329;
    int converted_val;

    initial begin
        $display("---------------------------------------------------");
        $display("Starting Zeta ROM Verification");
        $display("---------------------------------------------------");

        for (int i = 0; i < 128; i++) begin
            // Convert the signed expected value to unsigned mod 3329
            if (expected_signed[i] < 0) begin
                converted_val = q + expected_signed[i];
            end else begin
                converted_val = expected_signed[i];
            end

            // Compare against Hardware Table
            if (ZETA_MUL_TABLE[i] !== converted_val) begin
                $error("[FAIL] Index %0d: Expected %0d (Unsigned: %0d), Got %0d", 
                        i, expected_signed[i], converted_val, ZETA_MUL_TABLE[i]);
                errors++;
            end
        end

        if (errors == 0) begin
            $display("SUCCESS: All 128 Zeta Constants match FIPS 203 exactly.");
        end else begin
            $display("FAILURE: Found %0d mismatches.", errors);
        end
        $finish;
    end

endmodule
