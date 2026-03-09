"""
================================================================================
SCRIPT: Radix-4/2 NTT Hardware Pipeline Visualizer (Inha Uni Algorithm 5)
AUTHOR: Kiet Le
PURPOSE:
    This script "unrolls" the ML-KEM NTT by grouping array indices into
    hardware-ready execution packets following the specific scheduling
    of Algorithm 5. It simulates the UniPAM architecture's data-fetching
    stride pattern.

HARDWARE MAPPING:
    - Pass 1-3 (Radix-4): The "High-Throughput" mode. Data flows through
      PE0/PE2 into PE1/PE3. These passes handle the larger strides
      (e.g., len=128 down to 4).
    - Pass 4 (Radix-2): The "Odd Stage Out". This occurs at the VERY END
      of the NTT (len=2). Only PE0 and PE2 are active, processing
      contiguous blocks of 4 coefficients to simplify AGU logic.

ALGORITHM ALIGNMENT:
    - Matches Inha University Algorithm 5 (Mixed-Radix-4/2 NTT).
    - Optimized for a 4-bank Conflict-Free Memory Interface (CMI).
================================================================================
"""

import math

def unroll_ntt_radix42_to_file(filename="ntt_radix42_output.txt", N=16):
    """
    Unrolls the ML-KEM NTT algorithm into a Radix-4/2 hardware flow.
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
        print(f"--- Unrolling Radix-4/2 NTT Flow for N={N} ---", file=f)

        current_stage = 1
        pass_num = 1

        while current_stage <= total_stages:
            stages_remaining = total_stages - current_stage + 1

            # ---------------------------------------------------------
            # THE RADIX-2 PASS (Handles the odd stage out)
            # ---------------------------------------------------------
            if stages_remaining == 1:
                length = N // (2 ** current_stage)
                print(f"\n=== PASS {pass_num} (Radix-2): Stage {current_stage} (len = {length}) ===", file=f)

                num_blocks = 2 ** (current_stage - 1)
                for b in range(num_blocks):
                    start = b * 2 * length

                    # Calculate index 'i' for this block
                    i = (2 ** (current_stage - 1)) + b
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
                len_A = N // (2 ** stage_A)
                len_B = N // (2 ** stage_B)

                print(f"\n=== PASS {pass_num} (Radix-4): Stages {stage_A} & {stage_B} (len = {len_A} & {len_B}) ===", file=f)

                num_blocks = 2 ** (stage_A - 1)
                for b in range(num_blocks):
                    start = b * 2 * len_A

                    # Calculate the 3 'i' values used across the two stages for this macro-block
                    i_A     = (2 ** (stage_A - 1)) + b
                    i_B_top = (2 ** (stage_B - 1)) + (2 * b)
                    i_B_bot = (2 ** (stage_B - 1)) + (2 * b) + 1

                    z_A     = bit_rev(i_A)
                    z_B_top = bit_rev(i_B_top)
                    z_B_bot = bit_rev(i_B_bot)

                    print(f"  Macro-Block {b+1} (start = {start}): Uses zeta_A[{z_A}], zeta_B_top[{z_B_top}], zeta_B_bot[{z_B_bot}]", file=f)

                    for j in range(start, start + len_B):
                        # Hardware mapping: x0, x1, x2, x3
                        x0 = j
                        x1 = j + len_B
                        x2 = j + 2 * len_B
                        x3 = j + 3 * len_B

                        print(f"    Radix-4 Butterfly -> Inputs: f[{x0:2}], f[{x1:2}], f[{x2:2}], f[{x3:2}]", file=f)

                current_stage += 2
                pass_num += 1

    print(f"Output successfully saved to {filename}")

# Run the visualizer and save to file
unroll_ntt_radix42_to_file("ntt_radix42_output.txt", 256)
