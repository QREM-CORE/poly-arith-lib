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

    // =========================================================================
    // 3. Modular Reduction
    // =========================================================================
    parameter int Q_INV_NEG     = 3327;         // -Q^-1 mod R

endpackage : poly_arith_pkg
