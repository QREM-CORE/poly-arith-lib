# Polynomial Arithmetic Modules for ML-KEM

This repository contains high-performance, parameterizable SystemVerilog modules designed for polynomial arithmetic as specified in the FIPS 203 (ML-KEM) standard.

The library implements the fundamental mathematical building blocks required for Post-Quantum Cryptography (PQC), specifically targeting the Kyber/ML-KEM algorithm.

### Repository Scope
* Transformations: Fully compliant implementations of the Number Theoretic Transform (NTT) and Inverse NTT (INTT) using pre-computed Zeta constants.
* Pointwise Arithmetic: Hardware acceleration for Algorithm 11 (MultiplyNTTs) and Algorithm 12 (BaseCaseMultiply).
* Vector Operations: SIMD-style modules for polynomial addition and subtraction optimized for 256-bit AXI4-Stream interfaces.
* Modular Infrastructure: Signed Montgomery Reducers optimized for the $q = 3329$ field, utilizing 16-bit "Lazy Reduction" techniques to maximize DSP efficiency.
