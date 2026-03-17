/*
 * Module Name: rd_wr_addr_gen (Read/Write Address Generator)
 * Author(s): Mai Komar
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on "UniPAM" from:
 * H. Jung et al., "Highly-Efficient Hardware Architecture for ML-KEM PQC
 * Standard," IEEE Open Journal of Circuits and Systems, 2025.
 *
 * Description:
 * Generates 4 parallel polynomial coefficient indices per clock cycle
 * for conflict-free access to poly_mem_wrapper_4bank during NTT, INTT,
 * and CWM operations.
 *
 * This module is the coefficient-address counterpart to tf_addr_gen.
 * Both share the same FSM structure and pass schedule, and must be
 * driven by the same start_i / pass_idx_i control signals so the
 * coefficient and twiddle factor streams stay aligned.
 *
 * Conflict-Free Guarantee:
 *   poly_mem_wrapper_4bank uses bankIdx = sum of 2-bit chunks of index mod 4.
 *   Under this mapping, any group of 4 consecutive indices starting at a
 *   multiple of 4 always maps to all 4 distinct banks. This module exploits
 *   that property:
 *
 *   R4 passes: {base, base+stride, base+2*stride, base+3*stride} — the
 *   stride is always a power of 2 >= 4, so the four bankIdx values are
 *   always a permutation of {0,1,2,3}. No conflict.
 *
 *   R2 passes: Only 2 real butterfly indices are needed per cycle. To
 *   avoid the wrapper's conflict detector firing on the 2 unused lanes,
 *   dummy indices from the *complementary* pair in the same aligned group
 *   of 4 are output on lanes [3:2]. coeff_valid_o = 4'b0011 — the AU
 *   and CMI must only consume lanes [1:0] during R2 passes.
 *
 * NTT Stride Schedule (Cooley-Tukey DIT):
 *   Pass 1: R4, stride=64 | Pass 2: R4, stride=16
 *   Pass 3: R4, stride=4  | Pass 4: R2, stride=2
 *
 * INTT Stride Schedule (Gentleman-Sande DIF, reversed):
 *   Pass 1: R2, stride=2  | Pass 2: R4, stride=4
 *   Pass 3: R4, stride=16 | Pass 4: R4, stride=64
 */

import poly_arith_pkg::*;

module rd_wr_addr_gen (
    input  logic            clk,
    input  logic            rst,        // Active-high synchronous reset

    // ---- Control (identical interface to tf_addr_gen) ----
    input  logic            start_i,    // Pulse high for 1 cycle to begin a pass
    input  pe_mode_e        ctrl_i,     // NTT / INTT / CWM
    input  logic [1:0]      pass_idx_i, // Which pass to run (0..3)

    // ---- Coefficient index outputs ----
    // 4 parallel coefficient indices into the 256-element polynomial.
    // Indices are always conflict-free under poly_mem_wrapper's bankIdx mapping.
    output logic [3:0][7:0] coeff_idx_o,    // Coefficient indices 0-255
    output logic [3:0]      coeff_valid_o,  // Lane valid: 4'b1111 (R4/CWM), 4'b0011 (R2)

    // ---- Status (mirrors tf_addr_gen) ----
    output logic            valid_o,    // High while a pass is running
    output logic [1:0]      pass_o      // Current pass number (0-3)
);

    // =========================================================================
    // FSM State Encoding (identical to tf_addr_gen)
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE   = 3'b000,
        S_PASS_1 = 3'b001,
        S_PASS_2 = 3'b010,
        S_PASS_3 = 3'b011,
        S_PASS_4 = 3'b100
    } state_e;

    state_e     state_r, state_next;

    // =========================================================================
    // Internal Registers
    // =========================================================================
    logic       is_intt_r;          // Latched on start: 1 = INTT mode
    logic       is_cwm_r;           // Latched on start: 1 = CWM mode
    logic [5:0] bf_cnt_r;           // Butterfly counter within current block
    logic [5:0] block_cnt_r;        // Block counter within current pass

    // =========================================================================
    // Pass Configuration
    // =========================================================================
    logic [5:0] blocks_max;         // blocks_per_pass - 1
    logic [5:0] bfs_max;            // bfs_per_block - 1
    logic       pass_is_radix2;     // 1 = current pass is Radix-2
    logic [6:0] stride;             // Butterfly element stride (max 64)

    always_comb begin
        blocks_max     = '0;
        bfs_max        = '0;
        pass_is_radix2 = 1'b0;
        stride         = 7'd4;

        if (is_cwm_r) begin
            // CWM: 64 blocks x 1 BF, 4 sequential coefficients per clock
            blocks_max = 6'd63;
            bfs_max    = 6'd0;
            stride     = 7'd1;
        end else begin
            case (state_r)
                S_PASS_1: begin
                    if (is_intt_r) begin
                        // INTT Pass 1: R2, 64 blocks x 2 BFs, stride=2
                        blocks_max = 6'd63; bfs_max = 6'd1;
                        pass_is_radix2 = 1'b1; stride = 7'd2;
                    end else begin
                        // NTT Pass 1: R4, 1 block x 64 BFs, stride=64
                        blocks_max = 6'd0; bfs_max = 6'd63; stride = 7'd64;
                    end
                end
                S_PASS_2: begin
                    if (is_intt_r) begin
                        // INTT Pass 2: R4, 16 blocks x 4 BFs, stride=4
                        blocks_max = 6'd15; bfs_max = 6'd3; stride = 7'd4;
                    end else begin
                        // NTT Pass 2: R4, 4 blocks x 16 BFs, stride=16
                        blocks_max = 6'd3; bfs_max = 6'd15; stride = 7'd16;
                    end
                end
                S_PASS_3: begin
                    if (is_intt_r) begin
                        // INTT Pass 3: R4, 4 blocks x 16 BFs, stride=16
                        blocks_max = 6'd3; bfs_max = 6'd15; stride = 7'd16;
                    end else begin
                        // NTT Pass 3: R4, 16 blocks x 4 BFs, stride=4
                        blocks_max = 6'd15; bfs_max = 6'd3; stride = 7'd4;
                    end
                end
                S_PASS_4: begin
                    if (is_intt_r) begin
                        // INTT Pass 4: R4, 1 block x 64 BFs, stride=64
                        blocks_max = 6'd0; bfs_max = 6'd63; stride = 7'd64;
                    end else begin
                        // NTT Pass 4: R2, 64 blocks x 2 BFs, stride=2
                        blocks_max = 6'd63; bfs_max = 6'd1;
                        pass_is_radix2 = 1'b1; stride = 7'd2;
                    end
                end
                default: begin
                    blocks_max = '0; bfs_max = '0; stride = 7'd4;
                end
            endcase
        end
    end

    // =========================================================================
    // End-of-block / End-of-pass Detection
    // =========================================================================
    logic bf_last, block_last, pass_last;
    assign bf_last    = (bf_cnt_r == bfs_max);
    assign block_last = (block_cnt_r == blocks_max);
    assign pass_last  = bf_last & block_last;

    // =========================================================================
    // Coefficient Index Generation
    //
    // R4 butterfly (4 elements):
    //   base = block * (4 * stride) + bf_cnt
    //   idx  = {base, base+stride, base+2*stride, base+3*stride}
    //   Conflict-free because stride is always a power-of-2 >= 4.
    //
    // R2 butterfly (2 real elements + 2 dummy):
    //   group_base = block * 4  (aligned group of 4 consecutive coefficients)
    //   real   = {group_base+bf,   group_base+bf+2}   (stride=2 apart)
    //   dummy  = {group_base+1-bf, group_base+3-bf}   (the other pair)
    //   All 4 map to different banks → no conflict detected by wrapper.
    //   coeff_valid_o = 4'b0011 — AU and CMI consume only lanes [1:0].
    //
    // CWM (4 sequential elements):
    //   idx = {block*4, block*4+1, block*4+2, block*4+3}
    //   Conflict-free because any 4 consecutive indices from a 4-aligned
    //   start map to all 4 banks under the bankIdx sum formula.
    // =========================================================================

    // 10-bit intermediate to avoid overflow during R4 base computation
    // (e.g. block=3, stride=16 → 3 * 64 = 192; block=0, stride=64 → 0*256=0)
    logic [9:0] r4_base_wide;
    logic [7:0] r4_base;
    logic [7:0] r2_group_base;
    logic [7:0] r2_real0, r2_real1, r2_dum0, r2_dum1;

    always_comb begin
        // R4 base: block * (4*stride) + bf_cnt
        // For pass 1 (stride=64, block=0): 0*256+bf = bf  (no overflow)
        // For pass 2 (stride=16, block=0..3): block*64+bf  (max 207+15=222 < 256)
        // For pass 3 (stride=4,  block=0..15): block*16+bf (max 240+3=243 < 256)
        r4_base_wide = 10'(block_cnt_r) * 10'({stride[5:0], 2'b00}) + 10'(bf_cnt_r);
        r4_base      = r4_base_wide[7:0];

        // R2 group: block*4, bf selects which half of the group
        r2_group_base = {block_cnt_r[5:0], 2'b00};  // block * 4
        r2_real0 = r2_group_base + 8'(bf_cnt_r[0]);         // base + bf (0 or 1)
        r2_real1 = r2_group_base + 8'(bf_cnt_r[0]) + 8'd2;  // base + bf + 2
        r2_dum0  = r2_group_base + 8'(~bf_cnt_r[0] & 1'b1); // base + (1-bf)
        r2_dum1  = r2_group_base + 8'(~bf_cnt_r[0] & 1'b1) + 8'd2;

        if (state_r == S_IDLE) begin
            coeff_idx_o   = '0;
            coeff_valid_o = 4'b0000;
        end else if (is_cwm_r) begin
            // 4 sequential coefficients from current block
            coeff_idx_o[0] = {block_cnt_r[5:0], 2'b00};  // block*4 + 0
            coeff_idx_o[1] = {block_cnt_r[5:0], 2'b01};  // block*4 + 1
            coeff_idx_o[2] = {block_cnt_r[5:0], 2'b10};  // block*4 + 2
            coeff_idx_o[3] = {block_cnt_r[5:0], 2'b11};  // block*4 + 3
            coeff_valid_o  = 4'b1111;
        end else if (pass_is_radix2) begin
            // R2: real pair on lanes [1:0], conflict-avoiding dummies on [3:2]
            coeff_idx_o[0] = r2_real0;
            coeff_idx_o[1] = r2_real1;
            coeff_idx_o[2] = r2_dum0;
            coeff_idx_o[3] = r2_dum1;
            coeff_valid_o  = 4'b0011;
        end else begin
            // R4: standard butterfly pattern
            coeff_idx_o[0] = r4_base;
            coeff_idx_o[1] = r4_base + 8'(stride[6:0]);
            coeff_idx_o[2] = r4_base + 8'({stride[5:0], 1'b0});             // 2*stride
            coeff_idx_o[3] = r4_base + 8'({stride[5:0], 1'b0}) + 8'(stride[6:0]); // 3*stride
            coeff_valid_o  = 4'b1111;
        end
    end

    // =========================================================================
    // FSM: State Transition Logic (identical to tf_addr_gen)
    // =========================================================================
    always_comb begin
        state_next = state_r;
        case (state_r)
            S_IDLE: begin
                if (start_i) begin
                    if (ctrl_i == PE_MODE_NTT || ctrl_i == PE_MODE_INTT) begin
                        case (pass_idx_i)
                            2'd0: state_next = S_PASS_1;
                            2'd1: state_next = S_PASS_2;
                            2'd2: state_next = S_PASS_3;
                            2'd3: state_next = S_PASS_4;
                            default: state_next = S_IDLE;
                        endcase
                    end else if (ctrl_i == PE_MODE_CWM) begin
                        state_next = S_PASS_1;
                    end
                end
            end
            S_PASS_1, S_PASS_2, S_PASS_3, S_PASS_4:
                if (pass_last) state_next = S_IDLE;
            default: state_next = S_IDLE;
        endcase
    end

    // =========================================================================
    // FSM: Sequential Logic
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_r     <= S_IDLE;
            is_intt_r   <= 1'b0;
            is_cwm_r    <= 1'b0;
            bf_cnt_r    <= '0;
            block_cnt_r <= '0;
        end else begin
            state_r <= state_next;

            case (state_r)
                S_IDLE: begin
                    if (start_i) begin
                        is_intt_r   <= (ctrl_i == PE_MODE_INTT);
                        is_cwm_r    <= (ctrl_i == PE_MODE_CWM);
                        bf_cnt_r    <= '0;
                        block_cnt_r <= '0;
                    end
                end

                S_PASS_1, S_PASS_2, S_PASS_3, S_PASS_4: begin
                    if (pass_last) begin
                        bf_cnt_r    <= '0;
                        block_cnt_r <= '0;
                    end else if (bf_last) begin
                        bf_cnt_r    <= '0;
                        block_cnt_r <= block_cnt_r + 6'd1;
                    end else begin
                        bf_cnt_r <= bf_cnt_r + 6'd1;
                    end
                end

                default: begin
                    bf_cnt_r    <= '0;
                    block_cnt_r <= '0;
                end
            endcase
        end
    end

    // =========================================================================
    // Status Outputs
    // =========================================================================
    assign valid_o = (state_r != S_IDLE);

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