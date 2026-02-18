package poly_arith_pkg;
    // =========================================================================
    // 1. Bus & Interface Configuration (AXI4-Stream)
    // =========================================================================
    parameter int DWIDTH        = 256;          // Main data bus width (32 bytes)
    parameter int KEEP_WIDTH    = DWIDTH / 8;   // TKEEP width (32 bits)
    parameter int BYTE_SIZE     = 8;

    // =========================================================================
    // 2. ML-KEM Specific Constants (FIPS 203)
    // =========================================================================
    parameter int Q             = 3329;         // The Modulus
    parameter int N             = 256;          // Polynomial degree
    parameter int LOG_N         = 8;            // log2(256)

    // Coefficient sizing
    parameter int COEFF_WIDTH   = 12;           // Mathematical width (min needed)
    parameter int STORE_WIDTH   = 16;           // Storage width (aligned for hardware)

    // Throughput Calculations
    // How many coefficients fit in one AXI beat? (256 / 16 = 16 coefficients)
    parameter int COEFFS_PER_BEAT = DWIDTH / STORE_WIDTH;

    // How many beats to transfer one full polynomial? (256 / 16 = 16 beats)
    parameter int BEATS_PER_POLY  = N / COEFFS_PER_BEAT;

    typedef logic [11:0] coeff_t; // I/O type for coefficients (NTT and non-NTT values)

    // =========================================================================
    // 3. Zeta Pre-computed Values
    // =========================================================================

    // Algorithm 9/10: Forward/Inverse NTT Zeta Constants
    // Source: FIPS 203 Appendix A (Zeta^BitRev7(i) mod q)
    parameter logic [11:0] ZETA_NTT_TABLE [0:127] = '{
        12'd1,    12'd1729, 12'd2580, 12'd3289, 12'd2642, 12'd630,  12'd1897, 12'd848,
        12'd1062, 12'd1919, 12'd193,  12'd797,  12'd2786, 12'd3260, 12'd569,  12'd1746,
        12'd296,  12'd2447, 12'd1339, 12'd1476, 12'd3046, 12'd56,   12'd2240, 12'd1333,
        12'd1426, 12'd2094, 12'd535,  12'd2882, 12'd2393, 12'd2879, 12'd1974, 12'd821,
        12'd289,  12'd331,  12'd3253, 12'd1756, 12'd1197, 12'd2304, 12'd2277, 12'd2055,
        12'd650,  12'd1977, 12'd2513, 12'd632,  12'd2865, 12'd33,   12'd1320, 12'd1915,
        12'd2319, 12'd1435, 12'd807,  12'd452,  12'd1438, 12'd2868, 12'd1534, 12'd2402,
        12'd2647, 12'd2617, 12'd1481, 12'd648,  12'd2474, 12'd3110, 12'd1227, 12'd910,
        12'd17,   12'd2761, 12'd583,  12'd2649, 12'd1637, 12'd723,  12'd2288, 12'd1100,
        12'd1409, 12'd2662, 12'd3281, 12'd233,  12'd756,  12'd2156, 12'd3015, 12'd3050,
        12'd1703, 12'd1651, 12'd2789, 12'd1789, 12'd1847, 12'd952,  12'd1461, 12'd2687,
        12'd939,  12'd2308, 12'd2437, 12'd2388, 12'd733,  12'd2337, 12'd268,  12'd641,
        12'd1584, 12'd2298, 12'd2037, 12'd3220, 12'd375,  12'd2549, 12'd2090, 12'd1645,
        12'd1063, 12'd319,  12'd2773, 12'd757,  12'd2099, 12'd561,  12'd2466, 12'd2594,
        12'd2804, 12'd1092, 12'd403,  12'd1026, 12'd1143, 12'd2150, 12'd2775, 12'd886,
        12'd1722, 12'd1212, 12'd1874, 12'd1029, 12'd2110, 12'd2935, 12'd885,  12'd2154
    };

    // Algorithm 11: MultiplyNTTs Zeta Constants
    // Source: FIPS 203 Appendix A (Zeta^(2*BitRev7(i) + 1) mod q)
    // NOTE: Stored as unsigned 12-bit to save area. When used with signed coefficients,
    //       you MUST zero-extend and cast to signed: signed'({4'b0, val})
    parameter logic [11:0] ZETA_MUL_TABLE [0:127] = '{
        12'd17,   12'd3312, 12'd2761, 12'd568,  12'd583,  12'd2746, 12'd2649, 12'd680,
        12'd1637, 12'd1692, 12'd723,  12'd2606, 12'd2288, 12'd1041, 12'd1100, 12'd2229,
        12'd1409, 12'd1920, 12'd2662, 12'd667,  12'd3281, 12'd48,   12'd233,  12'd3096,
        12'd756,  12'd2573, 12'd2156, 12'd1173, 12'd3015, 12'd314,  12'd3050, 12'd279,
        12'd1703, 12'd1626, 12'd1651, 12'd1678, 12'd2789, 12'd540,  12'd1789, 12'd1540,
        12'd1847, 12'd1482, 12'd952,  12'd2377, 12'd1461, 12'd1868, 12'd2687, 12'd642,
        12'd939,  12'd2390, 12'd2308, 12'd1021, 12'd2437, 12'd892,  12'd2388, 12'd941,
        12'd733,  12'd2596, 12'd2337, 12'd992,  12'd268,  12'd3061, 12'd641,  12'd2688,
        12'd1584, 12'd1745, 12'd2298, 12'd1031, 12'd2037, 12'd1292, 12'd3220, 12'd109,
        12'd375,  12'd2954, 12'd2549, 12'd780,  12'd2090, 12'd1239, 12'd1645, 12'd1684,
        12'd1063, 12'd2266, 12'd319,  12'd3010, 12'd2773, 12'd556,  12'd757,  12'd2572,
        12'd2099, 12'd1230, 12'd561,  12'd2768, 12'd2466, 12'd863,  12'd2594, 12'd735,
        12'd2804, 12'd525,  12'd1092, 12'd2237, 12'd403,  12'd2926, 12'd1026, 12'd2303,
        12'd1143, 12'd2186, 12'd2150, 12'd1179, 12'd2775, 12'd554,  12'd886,  12'd2443,
        12'd1722, 12'd1607, 12'd1212, 12'd2117, 12'd1874, 12'd1455, 12'd1029, 12'd2300,
        12'd2110, 12'd1219, 12'd2935, 12'd394,  12'd885,  12'd2444, 12'd2154, 12'd1175
    };

    // =========================================================================
    // 4. PE Constants
    // =========================================================================

    // -------------------------------------------------------------------------
    // Processing Element (PE) Operating Modes
    // -------------------------------------------------------------------------
    typedef enum logic [3:0] {
        PE_MODE_CWM     = 4'b1000, // Coordinate-Wise Multiplication
        PE_MODE_NTT     = 4'b1010, // Number Theoretic Transform
        PE_MODE_INTT    = 4'b1111, // Inverse Number Theoretic Transform
        PE_MODE_ADDSUB  = 4'b0011, // Modular Addition / Subtraction
        PE_MODE_CODECO1 = 4'b1100, // Compression / Decompression (Variant 1)
        PE_MODE_CODECO2 = 4'b0100  // Compression / Decompression (Variant 2)
    } pe_mode_e;

endpackage : poly_arith_pkg
