/*
 * Module Name: tf_addr_gen (Twiddle Factor Address Generator)
 * Author(s): Salwan Aldhahab
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * Generates the ROM read addresses for the Twiddle Factor ROM (tf_rom) during
 * NTT, INTT, and CWM operations. This module implements the Conflict-Free
 * Address Generation Unit (AGU) for twiddle factors, producing the 3 ROM addresses
 * (w0, w1, w2) needed each cycle by the UniPAM butterfly array.
 *
 * The generator exploits a key mathematical property of the Mixed-Radix-4/2
 * NTT scheduling: within each macro-block, the twiddle factor addresses are
 * CONSTANT across all butterflies in that block. The addresses only change
 * at block boundaries. This allows a simple block-counter-based FSM to drive
 * the address generation, with combinational bit-reversal applied to produce
 * the final ROM indices.
 *
 * NTT Schedule (Forward, Cooley-Tukey, Decimation-in-Time):
 * | Pass | Type    | Stages | Blocks | BFs/Block | Total Cycles |
 * |------|---------|--------|--------|-----------|--------------|
 * | 1    | Radix-4 | 1 & 2  | 1      | 64        | 64           |
 * | 2    | Radix-4 | 3 & 4  | 4      | 16        | 64           |
 * | 3    | Radix-4 | 5 & 6  | 16     | 4         | 64           |
 * | 4    | Radix-2 | 7      | 64     | 2         | 128          |
 *
 * INTT Schedule (Inverse, Gentleman-Sande, Decimation-in-Frequency):
 * | Pass | Type    | Stages | Blocks | BFs/Block | Total Cycles |
 * |------|---------|--------|--------|-----------|--------------|
 * | 1    | Radix-2 | 1      | 64     | 2         | 128          |
 * | 2    | Radix-4 | 2 & 3  | 16     | 4         | 64           |
 * | 3    | Radix-4 | 4 & 5  | 4      | 16        | 64           |
 * | 4    | Radix-4 | 6 & 7  | 1      | 64        | 64           |
 *
 * CWM Schedule (Coordinate-Wise Multiplication):
 * | Pass | Type    | Blocks | BFs/Block | Total Cycles |
 * |------|---------|--------|-----------|--------------|
 * | 1    | BaseMul | 64     | 1         | 64           |
 *
 * Address Derivation (Radix-4 pass at NTT stages s_A, s_B = s_A+1):
 *   For block index b (0-based):
 *     i_A     = 2^(s_A - 1) + b
 *     i_B_top = 2^(s_B - 1) + 2*b
 *     i_B_bot = 2^(s_B - 1) + 2*b + 1
 *
 *     addr_w1 = BitRev7(i_A)      -> Stage A twiddle for PE2 mul1
 *     addr_w0 = BitRev7(i_B_top)  -> Stage B top twiddle for PE0
 *     addr_w2 = BitRev7(i_B_bot)  -> Stage B bot twiddle for PE2 mul2
 *
 *   Key identity: addr_w2 = addr_w0 | 7'b1000000 (differs only in MSB)
 *   Key identity: addr_w0 = BitRev7(i_A) << 1 (left shift of w1 addr)
 *
 * Latency: 0 clock cycles (combinational address output, registered in tf_rom)
 */

import poly_arith_pkg::*;

module tf_addr_gen (
    input   logic           clk,
    input   logic           rst,

    // ---- Control Interface (from AU Controller) ----
    input   logic           start_i,        // Pulse high for 1 cycle to begin
    input   pe_mode_e       mode_i,         // Operating mode (NTT, INTT, CWM)

    // ---- ROM Address Outputs (directly wired to tf_rom) ----
    output  logic [6:0]     addr0_o,        // ROM address for w0 (PE0)
    output  logic [6:0]     addr1_o,        // ROM address for w1 (PE2 mul1)
    output  logic [6:0]     addr2_o,        // ROM address for w2 (PE2 mul2)

    // ---- Status Outputs ----
    output  logic           valid_o,        // High when addresses are valid
    output  logic           busy_o,         // High during active generation
    output  logic           is_intt_o,      // Directly drives tf_rom is_intt_i
    output  logic [1:0]     pass_o          // Current pass number (0-3)
);

    // =========================================================================
    // FSM State Encoding
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE      = 3'b000,
        S_PASS_1    = 3'b001,
        S_PASS_2    = 3'b010,
        S_PASS_3    = 3'b011,
        S_PASS_4    = 3'b100
    } state_e;

    state_e state_r, state_next;

    // =========================================================================
    // Internal Registers
    // =========================================================================

    // Mode latch (captured on start)
    pe_mode_e   mode_r;
    logic       is_intt_r;
    logic       is_cwm_r;

    // Global cycle counter within current pass (max 128 for Radix-2 pass)
    logic [6:0] cycle_cnt_r;            // 0 to 127
    logic       cycle_cnt_last;         // Indicates last cycle of current pass

    // Block counter within current pass
    logic [5:0] block_cnt_r;            // 0 to 63

    // Butterfly counter within current block
    logic [5:0] bf_cnt_r;              // 0 to 63

    // Pass configuration (set by FSM on pass entry)
    logic [5:0] blocks_per_pass;        // Number of blocks in current pass
    logic [5:0] bfs_per_block;          // Butterflies per block in current pass
    logic       is_radix2;              // Current pass is Radix-2

    // =========================================================================
    // 7-Bit Reversal Function
    // =========================================================================
    // BitRev7(x) reverses the 7 bit positions of x.
    // Used to generate ROM addresses from sequential block indices.

    function automatic logic [6:0] bit_rev7 (input logic [6:0] val);
        bit_rev7 = {val[0], val[1], val[2], val[3], val[4], val[5], val[6]};
    endfunction

    // =========================================================================
    // Pass Configuration Logic
    // =========================================================================
    // Determines the block count and butterflies-per-block for each pass,
    // accounting for the NTT vs INTT pass ordering.
    //
    // NTT Pass Order:  R4(1&2) -> R4(3&4) -> R4(5&6) -> R2(7)
    // INTT Pass Order: R2(1) -> R4(2&3) -> R4(4&5) -> R4(6&7)

    // For NTT:
    //   Pass 1: 1 block,  64 BFs (R4, stages 1&2)
    //   Pass 2: 4 blocks, 16 BFs (R4, stages 3&4)
    //   Pass 3: 16 blocks, 4 BFs (R4, stages 5&6)
    //   Pass 4: 64 blocks, 2 BFs (R2, stage 7)
    //
    // For INTT:
    //   Pass 1: 64 blocks, 2 BFs (R2, stage 1)
    //   Pass 2: 16 blocks, 4 BFs (R4, stages 2&3)
    //   Pass 3: 4 blocks, 16 BFs (R4, stages 4&5)
    //   Pass 4: 1 block,  64 BFs (R4, stages 6&7)

    // Stage A base offset for Radix-4 passes (2^(s_A - 1))
    // This is the base 'i' value added to the block counter.
    logic [6:0] stg_a_base;

    always_comb begin
        blocks_per_pass = '0;
        bfs_per_block   = '0;
        is_radix2       = 1'b0;
        stg_a_base      = '0;

        if (is_cwm_r) begin
            // CWM: Single pass, 64 blocks × 1 BF
            blocks_per_pass = 6'd63;     // 64 blocks (0-indexed max)
            bfs_per_block   = 6'd0;      // 1 BF (0-indexed max)
            is_radix2       = 1'b0;
            stg_a_base      = 7'd64;
        end else begin
            case (state_r)
                S_PASS_1: begin
                    if (is_intt_r) begin
                        // INTT Pass 1: R2, stage 1, 64 blocks × 2 BFs
                        blocks_per_pass = 6'd63;
                        bfs_per_block   = 6'd1;
                        is_radix2       = 1'b1;
                    end else begin
                        // NTT Pass 1: R4, stages 1&2, 1 block × 64 BFs
                        blocks_per_pass = 6'd0;
                        bfs_per_block   = 6'd63;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd1;   // 2^(1-1) = 1
                    end
                end
                S_PASS_2: begin
                    if (is_intt_r) begin
                        // INTT Pass 2: R4, stages 2&3, 16 blocks × 4 BFs
                        blocks_per_pass = 6'd15;
                        bfs_per_block   = 6'd3;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd32;  // 2^(6-1) = 32 [INTT stage_A=6 equivalent]
                    end else begin
                        // NTT Pass 2: R4, stages 3&4, 4 blocks × 16 BFs
                        blocks_per_pass = 6'd3;
                        bfs_per_block   = 6'd15;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd4;   // 2^(3-1) = 4
                    end
                end
                S_PASS_3: begin
                    if (is_intt_r) begin
                        // INTT Pass 3: R4, stages 4&5, 4 blocks × 16 BFs
                        blocks_per_pass = 6'd3;
                        bfs_per_block   = 6'd15;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd8;   // 2^(4-1) = 8 [INTT stage_A=4 equivalent]
                    end else begin
                        // NTT Pass 3: R4, stages 5&6, 16 blocks × 4 BFs
                        blocks_per_pass = 6'd15;
                        bfs_per_block   = 6'd3;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd16;  // 2^(5-1) = 16
                    end
                end
                S_PASS_4: begin
                    if (is_intt_r) begin
                        // INTT Pass 4: R4, stages 6&7, 1 block × 64 BFs
                        blocks_per_pass = 6'd0;
                        bfs_per_block   = 6'd63;
                        is_radix2       = 1'b0;
                        stg_a_base      = 7'd2;   // 2^(2-1) = 2 [INTT stage_A=2 equivalent]
                    end else begin
                        // NTT Pass 4: R2, stage 7, 64 blocks × 2 BFs
                        blocks_per_pass = 6'd63;
                        bfs_per_block   = 6'd1;
                        is_radix2       = 1'b1;
                    end
                end
                default: begin
                    blocks_per_pass = '0;
                    bfs_per_block   = '0;
                    is_radix2       = 1'b0;
                    stg_a_base      = '0;
                end
            endcase
        end
    end

    // =========================================================================
    // Butterfly / Block Counter Logic
    // =========================================================================
    logic bf_cnt_last;
    logic block_cnt_last;

    assign bf_cnt_last    = (bf_cnt_r == bfs_per_block);
    assign block_cnt_last = (block_cnt_r == blocks_per_pass);
    assign cycle_cnt_last = bf_cnt_last & block_cnt_last;

    // =========================================================================
    // Address Computation
    // =========================================================================
    // Radix-4 NTT addressing:
    //   i_A     = stg_a_base + block_cnt_r
    //   i_B_top = 2 * stg_a_base + 2 * block_cnt_r       = (stg_a_base + block_cnt_r) << 1
    //   i_B_bot = 2 * stg_a_base + 2 * block_cnt_r + 1   = i_B_top | 1
    //
    //   addr_w1 = BitRev7(i_A)
    //   addr_w0 = BitRev7(i_B_top)
    //   addr_w2 = BitRev7(i_B_bot)
    //
    // Radix-2 NTT addressing:
    //   i = 64 + block_cnt_r
    //   addr_w0 = BitRev7(i)
    //
    // INTT Radix-4 addressing (Gentleman-Sande, reverse pass order):
    //   For INTT Pass 2 (stages 2&3, highest_i_A = 2^6-1 = 63):
    //     i_A_top = highest_i_A - 2*b    = (2*stg_a_base - 1) - 2*b
    //     i_A_bot = highest_i_A - 2*b -1 = (2*stg_a_base - 1) - 2*b - 1
    //     i_B     = highest_i_B - b      = (stg_a_base - 1) - b
    //
    //   The INTT uses the same ROM values but with Q-negation (handled in tf_rom).
    //   The INTT indices are derived by counting DOWN from the highest index.
    //
    // INTT Radix-2 addressing:
    //   highest_i = 127
    //   i = 127 - block_cnt_r
    //   addr_w0 = BitRev7(i)

    // --- Intermediate index values ---
    logic [6:0] i_A, i_B_top, i_B_bot;
    logic [6:0] i_radix2;

    // INTT helper: highest index values depend on the pass
    logic [6:0] intt_highest_i_A;   // (2 * stg_a_base) - 1
    logic [6:0] intt_highest_i_B;   // stg_a_base - 1

    assign intt_highest_i_A = (stg_a_base << 1) - 7'd1;
    assign intt_highest_i_B = stg_a_base - 7'd1;

    always_comb begin
        if (is_intt_r && !is_radix2) begin
            // INTT Radix-4: Count DOWN from highest index
            i_A     = intt_highest_i_B - block_cnt_r;                       // i_B in INTT = maps to w0
            i_B_top = intt_highest_i_A - {block_cnt_r, 1'b0};              // i_A_top in INTT = maps to w1
            i_B_bot = intt_highest_i_A - {block_cnt_r, 1'b0} - 7'd1;      // i_A_bot in INTT = maps to w2
        end else begin
            // NTT Radix-4: Count UP from base
            i_A     = stg_a_base + {1'b0, block_cnt_r};
            i_B_top = {stg_a_base[5:0], 1'b0} + {block_cnt_r, 1'b0};
            i_B_bot = {stg_a_base[5:0], 1'b0} + {block_cnt_r, 1'b0} + 7'd1;
        end

        // Radix-2 index
        if (is_intt_r) begin
            i_radix2 = 7'd127 - {1'b0, block_cnt_r};
        end else begin
            i_radix2 = 7'd64 + {1'b0, block_cnt_r};
        end
    end

    // --- CWM addressing ---
    // For CWM, we use the ZETA_MUL_TABLE (separate from NTT table).
    // The CWM twiddle factor index is simply the block counter (0..63).
    // However, CWM uses a DIFFERENT ROM (ZETA_MUL_TABLE), so the address
    // generation here outputs the block counter directly, and the top-level
    // controller routes to the appropriate ROM.
    // For simplicity, output block_cnt_r as addr0 during CWM.
    logic [6:0] cwm_addr;
    assign cwm_addr = {1'b0, block_cnt_r};

    // --- Final address generation ---
    always_comb begin
        if (state_r == S_IDLE) begin
            addr0_o = '0;
            addr1_o = '0;
            addr2_o = '0;
        end else if (is_cwm_r) begin
            // CWM: Only addr0 used (omega index = block counter)
            addr0_o = cwm_addr;
            addr1_o = '0;
            addr2_o = '0;
        end else if (is_radix2) begin
            // Radix-2: Only addr0 used
            addr0_o = bit_rev7(i_radix2);
            addr1_o = '0;
            addr2_o = '0;
        end else begin
            // Radix-4: All three ports active
            // For NTT: w0=Stage B top, w1=Stage A, w2=Stage B bot
            // For INTT: w0=Stage B (higher), w1=Stage A top, w2=Stage A bot
            addr0_o = bit_rev7(is_intt_r ? i_A : i_B_top);
            addr1_o = bit_rev7(is_intt_r ? i_B_top : i_A);
            addr2_o = bit_rev7(is_intt_r ? i_B_bot : i_B_bot);
        end
    end

    // =========================================================================
    // FSM: State Transition Logic
    // =========================================================================
    always_comb begin
        state_next = state_r;

        case (state_r)
            S_IDLE: begin
                if (start_i) begin
                    if (mode_i == PE_MODE_CWM) begin
                        // CWM only needs 1 pass
                        state_next = S_PASS_1;
                    end else if (mode_i == PE_MODE_NTT || mode_i == PE_MODE_INTT) begin
                        state_next = S_PASS_1;
                    end
                end
            end

            S_PASS_1: begin
                if (cycle_cnt_last) begin
                    if (is_cwm_r)
                        state_next = S_IDLE;
                    else
                        state_next = S_PASS_2;
                end
            end

            S_PASS_2: begin
                if (cycle_cnt_last)
                    state_next = S_PASS_3;
            end

            S_PASS_3: begin
                if (cycle_cnt_last)
                    state_next = S_PASS_4;
            end

            S_PASS_4: begin
                if (cycle_cnt_last)
                    state_next = S_IDLE;
            end

            default: state_next = S_IDLE;
        endcase
    end

    // =========================================================================
    // FSM: Sequential Logic
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_r     <= S_IDLE;
            mode_r      <= PE_MODE_NTT;
            is_intt_r   <= 1'b0;
            is_cwm_r    <= 1'b0;
            block_cnt_r <= '0;
            bf_cnt_r    <= '0;
            cycle_cnt_r <= '0;
        end else begin
            state_r <= state_next;

            case (state_r)
                S_IDLE: begin
                    if (start_i) begin
                        mode_r      <= mode_i;
                        is_intt_r   <= (mode_i == PE_MODE_INTT);
                        is_cwm_r    <= (mode_i == PE_MODE_CWM);
                        block_cnt_r <= '0;
                        bf_cnt_r    <= '0;
                        cycle_cnt_r <= '0;
                    end
                end

                S_PASS_1, S_PASS_2, S_PASS_3, S_PASS_4: begin
                    if (cycle_cnt_last) begin
                        // End of pass: reset counters for next pass
                        block_cnt_r <= '0;
                        bf_cnt_r    <= '0;
                        cycle_cnt_r <= '0;
                    end else begin
                        cycle_cnt_r <= cycle_cnt_r + 7'd1;

                        if (bf_cnt_last) begin
                            // End of block: advance to next block
                            bf_cnt_r    <= '0;
                            block_cnt_r <= block_cnt_r + 6'd1;
                        end else begin
                            bf_cnt_r <= bf_cnt_r + 6'd1;
                        end
                    end
                end

                default: begin
                    block_cnt_r <= '0;
                    bf_cnt_r    <= '0;
                    cycle_cnt_r <= '0;
                end
            endcase
        end
    end

    // =========================================================================
    // Status Outputs
    // =========================================================================
    assign busy_o    = (state_r != S_IDLE);
    assign valid_o   = (state_r != S_IDLE);
    assign is_intt_o = is_intt_r;

    always_comb begin
        case (state_r)
            S_PASS_1: pass_o = 2'd0;
            S_PASS_2: pass_o = 2'd1;
            S_PASS_3: pass_o = 2'd2;
            S_PASS_4: pass_o = 2'd3;
            default:  pass_o = 2'd0;
        endcase
    end

endmodule
