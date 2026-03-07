"""
================================================================================
SCRIPT: ML-KEM NTT (Algorithm 9) Software Flow Visualizer
AUTHOR: Kiet Le
PURPOSE:
    Provides a step-by-step unrolling of the standard Cooley-Tukey NTT
    algorithm used in ML-KEM (Kyber). This script demonstrates the
    sequential "Butterfly" operations across the 7 stages of the transform.

ALGORITHMIC FLOW (Algorithm 9):
    1. Outer Loop: Manages the 'len' distance between pairs (128 down to 2).
    2. Middle Loop: Moves through the polynomial in 'blocks'.
    3. Inner Loop: Performs the core Butterfly: A = A + (zeta * B) and
       B = A - (zeta * B).

USAGE:
    Use this script to verify the specific twiddle factor (zeta) indices and
    array index pairings required by the FIPS 203 specification before
    mapping them to parallel hardware.
================================================================================
"""

import math

def unroll_ntt_to_file(filename="ntt_unroll_output.txt", N=16):
    """
    Unrolls the ML-KEM NTT algorithm (Algorithm 9) to visualize the data flow
    and saves the output to a text file.
    N must be a power of 2. ML-KEM uses N=256.
    """
    # ML-KEM uses a 7-bit reversal for N=256. We adapt it based on N.
    bit_len = int(math.log2(N)) - 1

    # Open the file in write mode ('w')
    with open(filename, 'w') as f:
        print(f"--- Unrolling NTT Flow for N={N} ---", file=f)

        # Initialize variables matching Algorithm 9
        length = N // 2
        i = 1
        stage = 1

        # The Outer Loop: Stages
        while length >= 2:
            print(f"\n=== STAGE {stage} (len = {length}) ===", file=f)

            start = 0
            block = 1

            # The Middle Loop: Blocks
            while start < N:
                # Calculate Bit Reversal for the Twiddle Factor index
                # (Formats 'i' as binary, reverses it, converts back to integer)
                rev_i = int(f"{i:0{bit_len}b}"[::-1], 2) if bit_len > 0 else 0

                print(f"  Block {block} (start = {start}): Uses zeta[{rev_i}]", file=f)

                # The Inner Loop: Butterflies
                for j in range(start, start + length):
                    top_idx = j
                    bot_idx = j + length
                    print(f"    Butterfly -> Top: f[{top_idx:2}] interacting with Bottom: f[{bot_idx:2}]", file=f)

                # End of Block updates
                i += 1
                start += 2 * length
                block += 1

            # End of Stage updates
            length = length // 2
            stage += 1

    # Let the user know it finished successfully
    print(f"Output successfully saved to {filename}")

# Run the visualizer and save to file (Try changing 16 to 256!)
unroll_ntt_to_file("ntt_output.txt", 256)
