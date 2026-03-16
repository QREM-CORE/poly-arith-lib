import math

N = 256
Q = 3329
zeta = 17  # primitive 256th root of unity mod Q
bit_len = 7

def bit_rev7(val):
    return int(f"{val:07b}"[::-1], 2)

def mod_inv(a, m):
    """Modular inverse using Fermat's little theorem (m is prime)"""
    return pow(a, m - 2, m)

# Compute zeta^k mod Q for all k
zeta_pow = [1] * 256
for i in range(1, 256):
    zeta_pow[i] = (zeta_pow[i-1] * zeta) % Q

# The existing ZETA_NTT_TABLE: ZETA[i] = zeta^BitRev7(i) mod Q
ZETA = [0] * 128
for i in range(128):
    ZETA[i] = pow(zeta, bit_rev7(i), Q)

# Verify first few
assert ZETA[0] == 1
assert ZETA[1] == 1729
assert ZETA[64] == 17  # BitRev7(64)=1, zeta^1=17

print("=" * 70)
print("ROM CONTENT COMPUTATION FOR ML-KEM MIXED-RADIX-4/2 HARDWARE")
print("=" * 70)

# =====================================================================
# 1. R4NTT_ROM: 36-bit (3x12), sequential t++ addressing
#    Used for NTT Radix-4 passes (Alg 5, lines 2-14)
#    Content: {w1, w2, w3} standard positive twiddles
# =====================================================================

r4ntt_rom = []
t = 0
for p in range(3, 0, -1):
    i = 4 ** p
    n_4i = N // (4 * i)
    for k in range(n_4i):
        s_A = 2 * (4 - p) - 1  # stage numbers: p=3->1, p=2->3, p=1->5
        i_A = 2**(s_A - 1) + k
        i_B_top = 2 * i_A
        i_B_bot = 2 * i_A + 1
        
        w1 = ZETA[bit_rev7(i_A)]       # w1 (Stage A twiddle)
        w2 = ZETA[bit_rev7(i_B_top)]   # w2 (Stage B top twiddle)
        w3 = ZETA[bit_rev7(i_B_bot)]   # w3 (Stage B bottom twiddle)
        
        r4ntt_rom.append((w1, w2, w3))
        t += 1

print(f"\n1. R4NTT_ROM ({len(r4ntt_rom)} entries x 36-bit)")
print(f"   Format: {{w1[11:0], w2[11:0], w3[11:0]}}")
print(f"   Addressing: Sequential t++ counter")
print()

# Print pass boundaries
idx = 0
for p in range(3, 0, -1):
    i = 4 ** p
    n_4i = N // (4 * i)
    s_A = 2 * (4 - p) - 1
    s_B = s_A + 1
    print(f"   --- NTT Pass (p={p}, stages {s_A}&{s_B}): {n_4i} entries ---")
    for k in range(n_4i):
        w1, w2, w3 = r4ntt_rom[idx]
        print(f"   [{idx:2d}] w1={w1:4d}, w2={w2:4d}, w3={w3:4d}")
        idx += 1

# =====================================================================
# 2. w_ROM: 12-bit, addressed by j/4
#    Used for NTT Radix-2 final pass (Alg 5, lines 15-21)
#    Content: standard positive twiddles
# =====================================================================

omega_rom = []
for b in range(64):
    zeta_idx = bit_rev7(64 + b)
    omega_rom.append(ZETA[zeta_idx])

print(f"\n2. w_ROM ({len(omega_rom)} entries x 12-bit)")
print(f"   Addressing: j/4 (sequential 0..63)")
print()
for i in range(64):
    if i < 8 or i >= 56:
        print(f"   [{i:2d}] w={omega_rom[i]:4d}  (ZETA[BitRev7({64+i})] = ZETA[{bit_rev7(64+i)}])")
    elif i == 8:
        print(f"   ... (entries 8-55 omitted for brevity)")

# =====================================================================
# 3. R4INTT_ROM: 36-bit (3x12), sequential t++ addressing
#    Used for INTT Radix-4 passes (Alg 6, lines 8-21)
#    Content: PRE-NEGATED modular inverses: Q - z^-1
# =====================================================================

r4intt_rom = []
t = 0
for p in range(1, 4):  # p=1,2,3 (INTT goes forward in p)
    i = 4 ** p
    n_4i = N // (4 * i)
    
    stg_a_base_map = {1: 32, 2: 8, 3: 2}
    stg_a_base = stg_a_base_map[p]
    highest_i_A = 2 * stg_a_base - 1
    highest_i_B = stg_a_base - 1
    
    for k in range(n_4i):
        # INTT addressing (counting down):
        i_A_top = highest_i_A - 2 * k
        i_A_bot = highest_i_A - 2 * k - 1
        i_B = highest_i_B - k
        
        z_A_top = bit_rev7(i_A_top)
        z_A_bot = bit_rev7(i_A_bot)
        z_B = bit_rev7(i_B)
        
        ntt_w1 = ZETA[z_A_top]
        ntt_w2 = ZETA[z_B]
        ntt_w3 = ZETA[z_A_bot]
        
        inv_w1 = mod_inv(ntt_w1, Q)
        inv_w2 = mod_inv(ntt_w2, Q)
        inv_w3 = mod_inv(ntt_w3, Q)
        
        # Pre-negated
        pn_w1 = (Q - inv_w1) % Q
        pn_w2 = (Q - inv_w2) % Q
        pn_w3 = (Q - inv_w3) % Q
        
        r4intt_rom.append((pn_w1, pn_w2, pn_w3))
        t += 1

print(f"\n3. R4INTT_ROM ({len(r4intt_rom)} entries x 36-bit)")
print(f"   Format: {{w1_inv_neg[11:0], w2_inv_neg[11:0], w3_inv_neg[11:0]}}")
print(f"   Content: Pre-negated inverses (Q - z^-1)")
print(f"   Addressing: Sequential t++ counter")
print()

idx = 0
for p in range(1, 4):
    i = 4 ** p
    n_4i = N // (4 * i)
    print(f"   --- INTT Pass (p={p}, i={i}): {n_4i} entries ---")
    for k in range(n_4i):
        w1, w2, w3 = r4intt_rom[idx]
        print(f"   [{idx:2d}] w1_inv_neg={w1:4d}, w2_inv_neg={w2:4d}, w3_inv_neg={w3:4d}")
        idx += 1

# =====================================================================
# 4. w_inv_ROM: 12-bit, addressed by j/4
#    Used for INTT Radix-2 first pass (Alg 6, lines 1-7)
#    Content: PRE-NEGATED modular inverses: Q - z^-1
# =====================================================================

omega_inv_rom = []
for b in range(64):
    zeta_idx = bit_rev7(127 - b)
    ntt_val = ZETA[zeta_idx]
    inv_val = mod_inv(ntt_val, Q)
    pn_val = (Q - inv_val) % Q
    omega_inv_rom.append(pn_val)

print(f"\n4. w_inv_ROM ({len(omega_inv_rom)} entries x 12-bit)")
print(f"   Content: Pre-negated inverses (Q - z^-1)")
print(f"   Addressing: j/4 (sequential 0..63)")
print()
for i in range(64):
    if i < 8 or i >= 56:
        zidx = bit_rev7(127 - i)
        print(f"   [{i:2d}] w_inv_neg={omega_inv_rom[i]:4d}  (Q - inv(ZETA[{zidx}]))")
    elif i == 8:
        print(f"   ... (entries 8-55 omitted for brevity)")

# =====================================================================
# VERIFICATION: Cross-check a few values
# =====================================================================
print("\n" + "=" * 70)
print("VERIFICATION")
print("=" * 70)

# omega_4 values
omega_4 = pow(zeta, 64, Q)
omega_4_inv = mod_inv(omega_4, Q)
omega_4_inv_neg = (Q - omega_4_inv) % Q
print(f"\nw4^1 (NTT) = zeta^64 mod Q = {omega_4}")
print(f"w4^-1 (standard) = {omega_4_inv}")
print(f"w4^-1 pre-negated = Q - w4^-1 = {omega_4_inv_neg}")

print(f"\nw_ROM[0] = {omega_rom[0]} = ZETA[BitRev7(64)] = ZETA[{bit_rev7(64)}]")
print(f"w_inv_ROM[0] = {omega_inv_rom[0]} = Q - inv(ZETA[BitRev7(127)]) = Q - inv(ZETA[{bit_rev7(127)}])")

# R4NTT_ROM[0] should be the twiddles for NTT Pass 1 (p=3, stages 1&2, k=0)
print(f"\nR4NTT_ROM[0] = {{w1={r4ntt_rom[0][0]}, w2={r4ntt_rom[0][1]}, w3={r4ntt_rom[0][2]}}}")
print(f"  w1 = ZETA[BitRev7(1)] = ZETA[{bit_rev7(1)}] = {ZETA[bit_rev7(1)]}")
print(f"  w2 = ZETA[BitRev7(2)] = ZETA[{bit_rev7(2)}] = {ZETA[bit_rev7(2)]}")
print(f"  w3 = ZETA[BitRev7(3)] = ZETA[{bit_rev7(3)}] = {ZETA[bit_rev7(3)]}")

# Now output the ROM contents in SystemVerilog parameter format
print("\n" + "=" * 70)
print("SYSTEMVERILOG ROM PARAMETER DECLARATIONS")
print("=" * 70)

# R4NTT_ROM
print("\n// R4NTT_ROM: 21 entries x 36-bit {w1[11:0], w2[11:0], w3[11:0]}")
print("parameter logic [35:0] R4NTT_ROM [0:20] = '{")
for i, (w1, w2, w3) in enumerate(r4ntt_rom):
    comma = "," if i < len(r4ntt_rom) - 1 else ""
    print(f"    {{12'd{w1:4d}, 12'd{w2:4d}, 12'd{w3:4d}}}{comma}  // [{i:2d}]")
print("};")

# w_ROM
print("\n// OMEGA_ROM: 64 entries x 12-bit")
print("parameter logic [11:0] OMEGA_ROM [0:63] = '{")
lines = []
for i in range(0, 64, 8):
    vals = [f"12'd{omega_rom[j]:4d}" for j in range(i, min(i+8, 64))]
    comma = "," if i + 8 < 64 else ""
    lines.append("    " + ", ".join(vals) + comma)
print("\n".join(lines))
print("};")

# R4INTT_ROM
print("\n// R4INTT_ROM: 21 entries x 36-bit {w1_inv_neg[11:0], w2_inv_neg[11:0], w3_inv_neg[11:0]}")
print("parameter logic [35:0] R4INTT_ROM [0:20] = '{")
for i, (w1, w2, w3) in enumerate(r4intt_rom):
    comma = "," if i < len(r4intt_rom) - 1 else ""
    print(f"    {{12'd{w1:4d}, 12'd{w2:4d}, 12'd{w3:4d}}}{comma}  // [{i:2d}]")
print("};")

# w_inv_ROM
print("\n// OMEGA_INV_ROM: 64 entries x 12-bit (pre-negated)")
print("parameter logic [11:0] OMEGA_INV_ROM [0:63] = '{")
lines = []
for i in range(0, 64, 8):
    vals = [f"12'd{omega_inv_rom[j]:4d}" for j in range(i, min(i+8, 64))]
    comma = "," if i + 8 < 64 else ""
    lines.append("    " + ", ".join(vals) + comma)
print("\n".join(lines))
print("};")

# Double check: the total number of ROM entries
print(f"\n// Total ROM storage:")
print(f"//   R4NTT_ROM:    {len(r4ntt_rom):3d} x 36 = {len(r4ntt_rom)*36:5d} bits ({len(r4ntt_rom)*36/8:.0f} bytes)")
print(f"//   OMEGA_ROM:    {len(omega_rom):3d} x 12 = {len(omega_rom)*12:5d} bits ({len(omega_rom)*12/8:.0f} bytes)")
print(f"//   R4INTT_ROM:   {len(r4intt_rom):3d} x 36 = {len(r4intt_rom)*36:5d} bits ({len(r4intt_rom)*36/8:.0f} bytes)")
print(f"//   OMEGA_INV_ROM:{len(omega_inv_rom):3d} x 12 = {len(omega_inv_rom)*12:5d} bits ({len(omega_inv_rom)*12/8:.0f} bytes)")
total = len(r4ntt_rom)*36 + len(omega_rom)*12 + len(r4intt_rom)*36 + len(omega_inv_rom)*12
print(f"//   TOTAL:                 {total:5d} bits ({total/8:.0f} bytes)")
