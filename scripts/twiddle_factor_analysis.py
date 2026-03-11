import math

N = 256
Q = 3329
bit_len = int(math.log2(N)) - 1  # 7

def bit_rev7(val):
    return int(f"{val:07b}"[::-1], 2)

# The ZETA_NTT_TABLE from the package: zeta^BitRev7(i) mod q for i in 0..127
# First, compute what these values should be
# Primitive 256th root of unity mod 3329 is 17
zeta = 17  # primitive root

# Compute all zeta^k mod Q
zeta_powers = [1] * 256
for i in range(1, 256):
    zeta_powers[i] = (zeta_powers[i-1] * zeta) % Q

print("=== NTT TWIDDLE FACTOR ACCESS PATTERN (N=256) ===")
print()

# NTT Algorithm (Cooley-Tukey, decimation-in-time)
# For each pass, determine which zeta indices are needed

current_stage = 1
pass_num = 1
total_stages = 7

ntt_passes = []

while current_stage <= total_stages:
    stages_remaining = total_stages - current_stage + 1
    
    if stages_remaining == 1:
        # Radix-2 pass
        length = N // (2 ** current_stage)
        num_blocks = 2 ** (current_stage - 1)
        
        pass_info = {"type": "radix2", "pass": pass_num, "stage": current_stage, "len": length, "blocks": []}
        
        for b in range(num_blocks):
            start = b * 2 * length
            i = (2 ** (current_stage - 1)) + b
            zeta_idx = bit_rev7(i)
            pass_info["blocks"].append({"block": b, "start": start, "zeta_idx": zeta_idx, "butterflies": length})
        
        ntt_passes.append(pass_info)
        current_stage += 1
        pass_num += 1
    else:
        # Radix-4 pass (merges two stages)
        stage_A = current_stage
        stage_B = current_stage + 1
        len_A = N // (2 ** stage_A)
        len_B = N // (2 ** stage_B)
        
        pass_info = {"type": "radix4", "pass": pass_num, "stages": (stage_A, stage_B), 
                     "len_A": len_A, "len_B": len_B, "blocks": []}
        
        num_blocks = 2 ** (stage_A - 1)
        for b in range(num_blocks):
            start = b * 2 * len_A
            
            i_A     = (2 ** (stage_A - 1)) + b
            i_B_top = (2 ** (stage_B - 1)) + (2 * b)
            i_B_bot = (2 ** (stage_B - 1)) + (2 * b) + 1
            
            z_A     = bit_rev7(i_A)
            z_B_top = bit_rev7(i_B_top)
            z_B_bot = bit_rev7(i_B_bot)
            
            pass_info["blocks"].append({
                "block": b, "start": start,
                "z_A": z_A, "z_B_top": z_B_top, "z_B_bot": z_B_bot,
                "butterflies_per_block": len_B
            })
        
        ntt_passes.append(pass_info)
        current_stage += 2
        pass_num += 1

print("NTT Pass Summary:")
for p in ntt_passes:
    if p["type"] == "radix4":
        print(f"  Pass {p['pass']} (Radix-4): Stages {p['stages']}, len_A={p['len_A']}, len_B={p['len_B']}")
        print(f"    Blocks: {len(p['blocks'])}, Butterflies per block: {p['blocks'][0]['butterflies_per_block']}")
        print(f"    Twiddle indices per block:")
        for bl in p["blocks"]:
            print(f"      Block {bl['block']}: w1(stgA)=zeta[{bl['z_A']}], w0(stgB_top)=zeta[{bl['z_B_top']}], w2(stgB_bot)=zeta[{bl['z_B_bot']}]")
    else:
        print(f"  Pass {p['pass']} (Radix-2): Stage {p['stage']}, len={p['len']}")
        print(f"    Blocks: {len(p['blocks'])}, Butterflies per block: {p['blocks'][0]['butterflies']}")
        print(f"    Twiddle indices per block:")
        for bl in p["blocks"]:
            print(f"      Block {bl['block']}: w=zeta[{bl['zeta_idx']}]")

print()

# Now compute the COMPLETE sequence of twiddle factor ROM reads for NTT
print("=== COMPLETE NTT ROM ACCESS SEQUENCE ===")
print("(Each line = one group of coefficients pushed to the PE unit)")
print("Format: [cycle_within_pass] w0(PE0), w1(PE2_mul1), w2(PE2_mul2), w3(PE3_tf)")
print()

ntt_rom_sequence = []

for p in ntt_passes:
    if p["type"] == "radix4":
        for bl in p["blocks"]:
            # For each butterfly within the block, the twiddle factors are the SAME
            # They change per macro-block, not per butterfly
            for j in range(bl["butterflies_per_block"]):
                entry = {
                    "pass": p["pass"],
                    "block": bl["block"],
                    "butterfly": j,
                    "w0": bl["z_B_top"],   # w_2 (stage B top) -> PE0
                    "w1": bl["z_A"],       # w_1 (stage A) -> PE2 mul1
                    "w2": bl["z_B_bot"],   # w_3 (stage B bot) -> PE2 mul2
                }
                ntt_rom_sequence.append(entry)
    else:
        for bl in p["blocks"]:
            for j in range(bl["butterflies"]):
                entry = {
                    "pass": p["pass"],
                    "block": bl["block"],
                    "butterfly": j,
                    "w0": bl["zeta_idx"],  # Only w0 used in radix-2
                    "w1": 0,
                    "w2": 0,
                }
                ntt_rom_sequence.append(entry)

# Print unique per-block patterns
print(f"Total NTT cycles: {len(ntt_rom_sequence)}")
print()

# Extract the unique twiddle factor indices needed for NTT
ntt_unique_zetas = set()
for entry in ntt_rom_sequence:
    ntt_unique_zetas.add(entry["w0"])
    ntt_unique_zetas.add(entry["w1"])
    ntt_unique_zetas.add(entry["w2"])
ntt_unique_zetas.discard(0)  # Remove default
print(f"Unique NTT zeta indices: {sorted(ntt_unique_zetas)}")
print(f"Count: {len(ntt_unique_zetas)}")
print()

# Print the per-block access pattern (twiddle factors are constant within a block)
print("NTT Per-Block Twiddle Factor Pattern:")
for p in ntt_passes:
    print(f"  Pass {p['pass']}:")
    if p["type"] == "radix4":
        for bl in p["blocks"]:
            reps = bl["butterflies_per_block"]
            print(f"    Block {bl['block']:3d}: w0=zeta[{bl['z_B_top']:3d}], w1=zeta[{bl['z_A']:3d}], w2=zeta[{bl['z_B_bot']:3d}] (x{reps} cycles)")
    else:
        for bl in p["blocks"]:
            reps = bl["butterflies"]
            print(f"    Block {bl['block']:3d}: w0=zeta[{bl['zeta_idx']:3d}] (x{reps} cycles)")

# Now do the same for INTT
print()
print("=" * 60)
print("=== INTT TWIDDLE FACTOR ACCESS PATTERN (N=256) ===")
print()

current_stage = 1
pass_num = 1
intt_passes = []

while current_stage <= total_stages:
    stages_remaining = total_stages - current_stage + 1
    
    if current_stage == 1:
        # Radix-2 pass (INTT starts with radix-2)
        length = 2 ** current_stage  # len = 2
        num_blocks = 2 ** (total_stages - current_stage)  # 64
        highest_i = (2 ** (total_stages - current_stage + 1)) - 1  # 127
        
        pass_info = {"type": "radix2", "pass": pass_num, "stage": current_stage, "len": length, "blocks": []}
        
        for b in range(num_blocks):
            start = b * 2 * length
            i = highest_i - b
            zeta_idx = bit_rev7(i)
            pass_info["blocks"].append({"block": b, "start": start, "zeta_idx": zeta_idx, "butterflies": length})
        
        intt_passes.append(pass_info)
        current_stage += 1
        pass_num += 1
    else:
        # Radix-4 pass
        stage_A = current_stage
        stage_B = current_stage + 1
        len_A = 2 ** stage_A
        len_B = 2 ** stage_B
        
        pass_info = {"type": "radix4", "pass": pass_num, "stages": (stage_A, stage_B),
                     "len_A": len_A, "len_B": len_B, "blocks": []}
        
        num_blocks_B = 2 ** (total_stages - stage_B)
        highest_i_A = (2 ** (total_stages - stage_A + 1)) - 1
        highest_i_B = (2 ** (total_stages - stage_B + 1)) - 1
        
        for b in range(num_blocks_B):
            start = b * 2 * len_B
            
            i_A_top = highest_i_A - (2 * b)
            i_A_bot = highest_i_A - (2 * b + 1)
            i_B     = highest_i_B - b
            
            z_A_top = bit_rev7(i_A_top)
            z_A_bot = bit_rev7(i_A_bot)
            z_B     = bit_rev7(i_B)
            
            pass_info["blocks"].append({
                "block": b, "start": start,
                "z_A_top": z_A_top, "z_A_bot": z_A_bot, "z_B": z_B,
                "butterflies_per_block": len_A  # number of butterflies per block
            })
        
        intt_passes.append(pass_info)
        current_stage += 2
        pass_num += 1

print("INTT Pass Summary:")
for p in intt_passes:
    if p["type"] == "radix4":
        print(f"  Pass {p['pass']} (Radix-4): Stages {p['stages']}, len_A={p['len_A']}, len_B={p['len_B']}")
        print(f"    Blocks: {len(p['blocks'])}, Butterflies per block: {p['blocks'][0]['butterflies_per_block']}")
        for bl in p["blocks"]:
            print(f"      Block {bl['block']}: w1_inv(stgA_top)=zeta[{bl['z_A_top']}], w3_inv(stgA_bot)=zeta[{bl['z_A_bot']}], w2_inv(stgB)=zeta[{bl['z_B']}]")
    else:
        print(f"  Pass {p['pass']} (Radix-2): Stage {p['stage']}, len={p['len']}")
        print(f"    Blocks: {len(p['blocks'])}, Butterflies per block: {p['blocks'][0]['butterflies']}")
        for bl in p["blocks"][:5]:
            print(f"      Block {bl['block']}: w=zeta[{bl['zeta_idx']}]")
        print(f"      ... ({len(p['blocks'])} blocks total)")

print()
print("INTT Per-Block Twiddle Factor Pattern:")
for p in intt_passes:
    print(f"  Pass {p['pass']}:")
    if p["type"] == "radix4":
        for bl in p["blocks"]:
            reps = bl["butterflies_per_block"]
            print(f"    Block {bl['block']:3d}: w0=zeta[{bl['z_B']:3d}], w1=zeta[{bl['z_A_top']:3d}], w2=zeta[{bl['z_A_bot']:3d}] (x{reps} cycles)")
    else:
        for bl in p["blocks"][:5]:
            reps = bl["butterflies"]
            print(f"    Block {bl['block']:3d}: w0=zeta[{bl['zeta_idx']:3d}] (x{reps} cycles)")
        print(f"    ... ({len(p['blocks'])} blocks total)")

# Now compute the MERGED ROM content
# The paper uses a single ROM that stores zeta values indexed by BitRev7(i)
# The key insight: For NTT we need zeta^BitRev7(i) and for INTT we need zeta^(-BitRev7(i)) = -zeta^BitRev7(i) mod Q
# Since -x mod Q = Q - x, INTT just uses Q - NTT_value

# The ROM stores 128 entries of ZETA_NTT_TABLE
# The address generator produces the sequence of indices needed

print()
print("=" * 60)
print("=== ROM CONTENT VERIFICATION ===")
print()

# Verify the existing ZETA_NTT_TABLE values
ZETA_NTT_TABLE_expected = []
for i in range(128):
    val = pow(zeta, bit_rev7(i), Q)
    ZETA_NTT_TABLE_expected.append(val)

print("ZETA_NTT_TABLE (first 32 entries):")
for i in range(32):
    print(f"  [{i:3d}] = {ZETA_NTT_TABLE_expected[i]:4d}")

print()

# For INTT, the inverse twiddle factors: zeta^(-BitRev7(i)) mod Q
# = Q - zeta^BitRev7(i) mod Q (for non-zero values)
# So INTT uses (Q - ZETA_NTT_TABLE[i]) for each entry

# Now let's figure out the ADDRESS GENERATION PATTERN
# For each pass and block, what ROM address do we read?

print("=" * 60)
print("=== NTT ROM ADDRESS GENERATION SEQUENCE ===")
print("(Shows the ROM addresses read at the start of each block)")
print()

for p_idx, p in enumerate(ntt_passes):
    if p["type"] == "radix4":
        print(f"Pass {p['pass']} (Radix-4, Stages {p['stages']}):")
        addr_w1 = []
        addr_w0 = []
        addr_w2 = []
        for bl in p["blocks"]:
            addr_w1.append(bl["z_A"])
            addr_w0.append(bl["z_B_top"])
            addr_w2.append(bl["z_B_bot"])
            
        print(f"  w1 (Stage A) ROM addrs: {addr_w1}")
        print(f"  w0 (Stage B top) ROM addrs: {addr_w0}")
        print(f"  w2 (Stage B bot) ROM addrs: {addr_w2}")
        print()
    else:
        print(f"Pass {p['pass']} (Radix-2, Stage {p['stage']}):")
        addrs = [bl["zeta_idx"] for bl in p["blocks"]]
        print(f"  w0 ROM addrs: {addrs}")
        print()

print()
print("=" * 60)
print("=== INTT ROM ADDRESS GENERATION SEQUENCE ===")
print()

for p_idx, p in enumerate(intt_passes):
    if p["type"] == "radix4":
        print(f"Pass {p['pass']} (Radix-4, Stages {p['stages']}):")
        for bl in p["blocks"]:
            print(f"  Block {bl['block']}: w0(stgB)=ROM[{bl['z_B']}], w1(stgA_top)=ROM[{bl['z_A_top']}], w2(stgA_bot)=ROM[{bl['z_A_bot']}]")
        print()
    else:
        print(f"Pass {p['pass']} (Radix-2, Stage {p['stage']}):")
        for bl in p["blocks"][:5]:
            print(f"  Block {bl['block']}: w0=ROM[{bl['zeta_idx']}]")
        print(f"  ... ({len(p['blocks'])} blocks total)")
        addrs = [bl["zeta_idx"] for bl in p["blocks"]]
        print(f"  w0 ROM addrs: {addrs}")
        print()

# KEY INSIGHT: Let's analyze the ADDRESS PATTERN more carefully
# to find a hardware-efficient generation scheme

print("=" * 60)
print("=== ADDRESS PATTERN ANALYSIS ===")
print()

# For NTT Pass 1 (Stages 1&2): 1 macro-block, 64 butterflies
# i_A = 1, i_B_top = 2, i_B_bot = 3
# z_A = BitRev7(1) = 64, z_B_top = BitRev7(2) = 32, z_B_bot = BitRev7(3) = 96

# For NTT Pass 2 (Stages 3&4): 4 macro-blocks, 16 butterflies each
# For NTT Pass 3 (Stages 5&6): 16 macro-blocks, 4 butterflies each
# For NTT Pass 4 (Stage 7): 64 blocks, 1 butterfly each

print("NTT Radix-4 Block count per pass:")
for p in ntt_passes:
    if p["type"] == "radix4":
        print(f"  Pass {p['pass']}: {len(p['blocks'])} blocks, {p['blocks'][0]['butterflies_per_block']} butterflies/block")
    else:
        print(f"  Pass {p['pass']}: {len(p['blocks'])} blocks, {p['blocks'][0]['butterflies']} butterflies/block")

print()

# Now let's see the sequential ROM address sequence for all 256 cycles
print("FULL NTT ROM ADDRESS SEQUENCE (cycle-by-cycle, 256 total):")
cycle = 0
for p in ntt_passes:
    if p["type"] == "radix4":
        for bl in p["blocks"]:
            for j in range(bl["butterflies_per_block"]):
                if cycle < 10 or (cycle > 62 and cycle < 68) or (cycle > 126 and cycle < 133) or cycle > 252:
                    print(f"  Cycle {cycle:3d}: Pass {p['pass']}, Block {bl['block']:3d}, BF {j:2d} -> w0=ROM[{bl['z_B_top']:3d}], w1=ROM[{bl['z_A']:3d}], w2=ROM[{bl['z_B_bot']:3d}]")
                elif cycle == 10 or cycle == 68 or cycle == 133:
                    print(f"  ...")
                cycle += 1
    else:
        for bl in p["blocks"]:
            for j in range(bl["butterflies"]):
                if cycle < 10 or cycle > 252:
                    print(f"  Cycle {cycle:3d}: Pass {p['pass']}, Block {bl['block']:3d}, BF {j:2d} -> w0=ROM[{bl['zeta_idx']:3d}]")
                elif cycle == 10:
                    print(f"  ...")
                cycle += 1

print()
print(f"Total cycles: {cycle}")

# What's the omega_4^1 value?
# For the radix-4 butterfly, w3 = zeta^(N/4) = zeta^64 = 17^64 mod 3329
omega_4 = pow(zeta, 64, Q)
print(f"\nomega_4^1 (zeta^64 mod Q) = {omega_4}")
# And its inverse for INTT
omega_4_inv = pow(omega_4, Q-2, Q)
print(f"omega_4^(-1) mod Q = {omega_4_inv}")

# Verify: omega_4^2 should be -1 mod Q = Q-1
omega_4_sq = pow(omega_4, 2, Q)
print(f"omega_4^2 mod Q = {omega_4_sq} (should be {Q-1})")

# Now let's generate the INTT sequence of ROM addresses for a cycle-by-cycle schedule too
print()
print("FULL INTT ROM ADDRESS SEQUENCE (cycle-by-cycle, 256 total):")
cycle = 0
for p in intt_passes:
    if p["type"] == "radix2":
        for bl in p["blocks"]:
            for j in range(bl["butterflies"]):
                if cycle < 10 or cycle > 124:
                    print(f"  Cycle {cycle:3d}: Pass {p['pass']}, Block {bl['block']:3d}, BF {j:2d} -> w0=ROM[{bl['zeta_idx']:3d}]")
                elif cycle == 10:
                    print(f"  ...")
                cycle += 1
    else:
        for bl in p["blocks"]:
            for j in range(bl["butterflies_per_block"]):
                if cycle < 136 and cycle > 126:
                    print(f"  Cycle {cycle:3d}: Pass {p['pass']}, Block {bl['block']:3d}, BF {j:2d} -> w0=ROM[{bl['z_B']:3d}], w1=ROM[{bl['z_A_top']:3d}], w2=ROM[{bl['z_A_bot']:3d}]")
                elif cycle == 136:
                    print(f"  ...")
                elif cycle > 252:
                    print(f"  Cycle {cycle:3d}: Pass {p['pass']}, Block {bl['block']:3d}, BF {j:2d} -> w0=ROM[{bl['z_B']:3d}], w1=ROM[{bl['z_A_top']:3d}], w2=ROM[{bl['z_A_bot']:3d}]")
                cycle += 1

print(f"\nTotal INTT cycles: {cycle}")
