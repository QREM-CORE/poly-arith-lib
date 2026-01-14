# ML-KEM Modular Reduction LUT Generator
#
# [cite_start]Based on the method described in:
# H. Jung, Q. Dang Truong and H. Lee, "Highly-Efficient Hardware Architecture for ML-KEM PQC Standard,"
# in IEEE Open Journal of Circuits and Systems, vol. 6, pp. 356-369, 2025
#
# The goal is to reduce a 24-bit product (from 12-bit x 12-bit mult) modulo 3329.
# Formula: Product = (A * 2^20) + (B * 2^16) + (C * 2^12) + D
# We pre-calculate (Value * 2^Shift) % 3329 for chunks A, B, and C.

q = 3329
weights = {
    "LUT_23_20": (2**20) % q, # 3270
    "LUT_19_16": (2**16) % q, # 2285
    "LUT_15_12": (2**12) % q  # 767
}

for name, w in weights.items():
    print(f"// {name} Generation")
    limit = 11 if "23_20" in name else 16
    for i in range(limit):
        val = (i * w) % q
        print(f"4'd{i}: out = 12'd{val};")
