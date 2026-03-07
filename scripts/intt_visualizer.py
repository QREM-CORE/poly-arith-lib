"""
================================================================================
SCRIPT: ML-KEM INTT (Algorithm 10) Software Flow Visualizer
AUTHOR: Kiet Le
PURPOSE:
    Provides a step-by-step unrolling of the Inverse Number Theoretic
    Transform (INTT) used in ML-KEM (Kyber). This script demonstrates the
    sequential Gentleman-Sande "Butterfly" operations across the stages.

ALGORITHMIC FLOW (Algorithm 10):
    1. Outer Loop: Manages the 'len' distance between pairs (starts at 2,
       doubles up to 128).
    2. Middle Loop: Moves through the polynomial in 'blocks'.
    3. Inner Loop: Performs the core Butterfly: Top = Top + Bottom and
       Bottom = zeta * (Bottom - Top).
    4. Final Step: Multiplies every entry by 3303 (the modular inverse of 256).

USAGE:
    Use this script to verify the specific twiddle factor (zeta) indices and
    array index pairings required by the FIPS 203 specification before
    mapping them to your Radix-4/2 parallel hardware.
================================================================================
"""

import math

def unroll_intt_to_file(filename="intt_unroll_output.txt", N=16):
    """
    Unrolls the ML-KEM INTT algorithm (Algorithm 10) to visualize the data flow
    and saves the output to a text file.
    N must be a power of 2. ML-KEM uses N=256.
    """
    # ML-KEM uses a 7-bit reversal for N=256. We adapt it based on N.
    bit_len = int(math.log2(N)) - 1

    # Open the file in write mode ('w')
    with open(filename, 'w') as f:
        print(f"--- Unrolling INTT Flow for N={N} ---", file=f)

        # Initialize variables matching Algorithm 10
        length = 2
        i = (N // 2) - 1
        stage = 1

        # The Outer Loop: Stages (doubling distance)
        while length <= N // 2:
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
                i -= 1  # INTT counts backwards!
                start += 2 * length
                block += 1

            # End of Stage updates
            length = length * 2
            stage += 1

        # The Final Scaling Step (Line 14 in Algorithm 10)
        print(f"\n=== FINAL SCALING ===", file=f)
        print(f"  Multiply all entries f[0] to f[{N-1}] by 3303 mod q", file=f)

    # Let the user know it finished successfully
    print(f"Output successfully saved to {filename}")

# Run the visualizer and save to file
unroll_intt_to_file("intt_output.txt", 256)
