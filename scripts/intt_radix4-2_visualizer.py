"""
================================================================================
SCRIPT: Radix-4/2 INTT Hardware Pipeline Visualizer
AUTHOR: Kiet Le
PURPOSE:
    This script "unrolls" the ML-KEM INTT (Inverse Number Theoretic Transform)
    by grouping array indices into hardware-ready execution packets. It maps
    the Gentleman-Sande butterflies to the UniPAM architecture.

HARDWARE MAPPING:
    - Pass (Radix-4): The "High-Throughput" mode. Data flows through PE1/PE3
      (Stage A, smaller len) directly into PE2/PE0 (Stage B, larger len) via
      internal feedback wires, skipping a memory write-back cycle.
    - Pass (Radix-2): Represents the "odd stage out". In the INTT, this is
      always the FINAL stage (len=128 for ML-KEM) where only one column of
      PEs is utilized to finish the algorithm.

INPUTS:
    - N: Polynomial degree (Default 16 for clarity; 256 for ML-KEM standard).
    - filename: The target text file to save the execution roadmap.
================================================================================
"""

import math

def unroll_intt_radix42_to_file(filename="intt_radix42_output.txt", N=16):
    """
    Unrolls the ML-KEM INTT algorithm into a Radix-4/2 hardware flow.
    Saves the pipeline groupings to a text file.
    N must be a power of 2. ML-KEM uses N=256.
    """
    # ML-KEM uses a 7-bit reversal for N=256. We adapt it based on N.
    bit_len = int(math.log2(N)) - 1
    total_stages = bit_len

    # Helper function for Bit Reversal
    def bit_rev(val):
        return int(f"{val:0{bit_len}b}"[::-1], 2) if bit_len > 0 else 0

    with open(filename, 'w') as f:
        print(f"--- Unrolling Radix-4/2 INTT Flow for N={N} ---", file=f)

        current_stage = 1
        pass_num = 1

        while current_stage <= total_stages:
            stages_remaining = total_stages - current_stage + 1

            # ---------------------------------------------------------
            # THE RADIX-2 PASS (Handles the odd stage out at the END)
            # ---------------------------------------------------------
            if stages_remaining == 1 and total_stages % 2 != 0:
                length = 2 ** current_stage
                print(f"\n=== PASS {pass_num} (Radix-2): Stage {current_stage} (len = {length}) ===", file=f)

                num_blocks = 2 ** (total_stages - current_stage)
                highest_i = (2 ** (total_stages - current_stage + 1)) - 1

                for b in range(num_blocks):
                    start = b * 2 * length

                    # Calculate index 'i' (INTT counts downwards)
                    i = highest_i - b
                    zeta_idx = bit_rev(i)

                    print(f"  Block {b+1} (start = {start}): Uses zeta[{zeta_idx}]", file=f)

                    for j in range(start, start + length):
                        # Hardware mapping: x0 -> Top, x2 -> Bottom
                        print(f"    Radix-2 Butterfly -> Inputs: f[{j:2}] & f[{j+length:2}]", file=f)

                current_stage += 1
                pass_num += 1

            # ---------------------------------------------------------
            # THE RADIX-4 PASS (Stitches two stages together)
            # ---------------------------------------------------------
            else:
                stage_A = current_stage
                stage_B = current_stage + 1
                len_A = 2 ** stage_A
                len_B = 2 ** stage_B

                print(f"\n=== PASS {pass_num} (Radix-4): Stages {stage_A} & {stage_B} (len = {len_A} & {len_B}) ===", file=f)

                num_blocks_B = 2 ** (total_stages - stage_B)
                highest_i_A = (2 ** (total_stages - stage_A + 1)) - 1
                highest_i_B = (2 ** (total_stages - stage_B + 1)) - 1

                for b in range(num_blocks_B):
                    start = b * 2 * len_B

                    # Calculate the 3 'i' values used across the two stages (Counting downwards)
                    i_A_top = highest_i_A - (2 * b)
                    i_A_bot = highest_i_A - (2 * b + 1)
                    i_B     = highest_i_B - b

                    z_A_top = bit_rev(i_A_top)
                    z_A_bot = bit_rev(i_A_bot)
                    z_B     = bit_rev(i_B)

                    print(f"  Macro-Block {b+1} (start = {start}): Uses zeta_A_top[{z_A_top}], zeta_A_bot[{z_A_bot}], zeta_B[{z_B}]", file=f)

                    for j in range(start, start + len_A):
                        # Hardware mapping: x0, x1, x2, x3 mapped to len_A spacing
                        x0 = j
                        x1 = j + len_A
                        x2 = j + 2 * len_A
                        x3 = j + 3 * len_A

                        print(f"    Radix-4 Butterfly -> Inputs: f[{x0:2}], f[{x1:2}], f[{x2:2}], f[{x3:2}]", file=f)

                current_stage += 2
                pass_num += 1

        # The Final Scaling Step
        print(f"\n=== FINAL SCALING ===", file=f)
        print(f"  Multiply all entries f[0] to f[{N-1}] by 3303 mod q", file=f)

    print(f"Output successfully saved to {filename}")

# Run the visualizer and save to file
unroll_intt_radix42_to_file("intt_radix42_output.txt", 256)
