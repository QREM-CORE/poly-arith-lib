# ML-KEM Modular Reduction LUT Generator
#
# [cite_start]Based on the method described in:
# H. Jung, Q. Dang Truong and H. Lee, "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"
# in IEEE Open Journal of Circuits and Systems, vol. 6, pp. 356-369, 2025
#
# The goal is to reduce a 24-bit product (from 12-bit x 12-bit mult) modulo 3329.
# Formula: Product = (A * 2^20) + (B * 2^16) + (C * 2^12) + D
# We pre-calculate (Value * 2^Shift) % 3329 for chunks A, B, and C.

# ML-KEM Modular Reduction LUT Generator
# Based on the method described in "Highly-Efficient Hardware Architecture for ML-KEM"
#
# Updated to support 26-bit inputs (required for Karatsuba intermediate sums).
# Formula: Product = (High * 2^24) + (A * 2^20) + (B * 2^16) + (C * 2^12) + D

q = 3329

# 1. Define weights for the standard 4-bit chunks
weights_4bit = {
    "LUT_23_20": (2**20) % q, # 3270
    "LUT_19_16": (2**16) % q, # 2285
    "LUT_15_12": (2**12) % q  # 767
}

print("// ==========================================")
print("// 4-BIT CHUNK GENERATION (Bits 0-23)")
print("// ==========================================\n")

for name, w in weights_4bit.items():
    print(f"// {name} Generation (Weight: {w})")
    print(f"case(chunk_{name[-5:]})")
    
    # Updated: loop up to 16 for ALL chunks to handle Karatsuba overflow safely
    for i in range(16):
        val = (i * w) % q
        print(f"    4'd{i:<2}: out = 12'd{val};")
        
    print("    default: out = 12'd0;")
    print("endcase\n")


# 2. Define weights for the new high bits (25:24)
# Note: Input is a 2-bit chunk (0-3)
weight_24 = (2**24) % q # 2385
weight_25 = (2**25) % q # 1441

print("// ==========================================")
print("// 2-BIT HIGH CHUNK GENERATION (Bits 25:24)")
print("// ==========================================\n")
print(f"// Weight Bit 24: {weight_24}")
print(f"// Weight Bit 25: {weight_25}")
print("case(chunk_high)")

for i in range(4):
    # Calculate value based on active bits
    val = 0
    if (i & 1): val += weight_24 # Bit 0 of chunk is Bit 24 of input
    if (i & 2): val += weight_25 # Bit 1 of chunk is Bit 25 of input
    
    val = val % q
    print(f"    2'd{i}: high_bit_val = 12'd{val};")

print("    default: high_bit_val = 12'd0;")
print("endcase")
