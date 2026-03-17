/*
 * Module Name: unipam_controller
 * Author(s): Jessica Buentipo
 * Target: FIPS 203 (ML-KEM / Kyber) Hardware Accelerator
 *
 * Reference:
 * Architecture based on the "Unified Polynomial Arithmetic Module (UniPAM)" from:
 * H. Jung, Q. D. Truong and H. Lee, "Highly-Efficient Hardware Architecture
 * for ML-KEM PQC Standard," in IEEE Open Journal of Circuits and Systems, 2025,
 * doi: 10.1109/OJCAS.2025.3591136. (Inha University)
 *
 * Description:
 * The unipam_controller is the central control unit of UNIPAM. It orchestrates data movement, computation scheduling,
 * and pipeline synchronization across the PEs, memory, and tf subsystem.
 *
 * Functionality:
 * 1. UNIPAM controller will begin execution when start_i is asserted. start_i to be received from main controller.
 * 2. op_type_i will be latched at this moment. op_type_i also to be received from main controller.
 * 3. Supported modes include NTT, INTT, CWM, Compression/Decompression, Add/Sub. Operation type pe_ctrl_o sent to pe_unit.
 *
 */

import poly_arith_pkg::*;

module unipam_controller (
    input  logic             clk,
    input  logic             rst,

    // Top-level control signals
    input  logic             start_i,       // Start signal
    input  pe_mode_e         op_type_i,     // PE_MODE_NTT / INTT / CWM / ADDSUB / COMP / DECOMP

    output logic             ready_o,
    output logic             done_o,

    // Interface to twiddle address generator / ROM
    output logic             tf_start_o,
    output logic [1:0]       pass_idx_o,
    output logic             tf_is_intt_o,

    // Interface to PE unit
    output pe_mode_e         pe_ctrl_o,
    output logic             pe_valid_o,

    input  logic             pe0_valid_i,
    input  logic             pe1_valid_i,
    input  logic             pe2_valid_i,
    input  logic             pe2_valid_m_i,
    input  logic             pe3_valid_i,

    // Interface to memory / CMI
    output logic             mem_read_en_o,
    output logic             mem_write_en_o,
    output logic [7:0]       rAddr_o,
    output logic [7:0]       wAddr_o,

    // ---- Optional pass status ----
    output logic [5:0]       block_cnt_o,
    output logic [5:0]       bf_cnt_o
);

    // =========================================================================
    // FSM States
    // =========================================================================
    typedef enum logic [2:0] {
        S_IDLE      = 3'd0,
        S_SETUP     = 3'd1,
        S_RUN       = 3'd2,
        S_DRAIN     = 3'd3,
        S_NEXT_PASS = 3'd4,
        S_DONE      = 3'd5
    } state_e;

    state_e state_r, state_n;

    // =========================================================================
    // Latched operation + current pass
    // =========================================================================
    pe_mode_e      op_r;
    logic [1:0]    pass_idx_r, pass_idx_n;

    // =========================================================================
    // Current-pass configuration
    // =========================================================================
    logic          pass_is_radix2;
    logic [5:0]    blocks_max;
    logic [5:0]    bfs_max;
    logic [2:0]    pipe_lat;
    logic          pass_uses_tf;
    logic          last_pass;

    // =========================================================================
    // Per-pass counters
    // =========================================================================
    logic [5:0]    block_cnt_r, block_cnt_n;
    logic [5:0]    bf_cnt_r,    bf_cnt_n;

    logic          bf_last;
    logic          block_last;
    logic          issue_last;

    // =========================================================================
    // Drain control
    // =========================================================================
    logic [2:0]    drain_cnt_r, drain_cnt_n;
    logic          all_pe_idle;
    logic          drain_done;

    // =========================================================================
    // Read issue counter and delayed writeback alignment
    // =========================================================================
    logic [7:0] issue_addr_r, issue_addr_n;

    logic [7:0] raddr_pipe [0:3];
    logic       wen_pipe   [0:3];

    integer i;

    // =========================================================================
    // Pass configuration
    // =========================================================================
    always_comb begin
        pass_is_radix2 = 1'b0;
        blocks_max     = 6'd0;
        bfs_max        = 6'd0;
        pipe_lat       = 3'd1;
        pass_uses_tf   = 1'b0;
        last_pass      = 1'b1;

        unique case (op_r)

            PE_MODE_NTT: begin
                pipe_lat     = 3'd4;
                pass_uses_tf = 1'b1;
                last_pass    = (pass_idx_r == 2'd3);
                unique case (pass_idx_r)
                    2'd0: begin pass_is_radix2 = 1'b0; blocks_max = 6'd0;  bfs_max = 6'd63; end
                    2'd1: begin pass_is_radix2 = 1'b0; blocks_max = 6'd3;  bfs_max = 6'd15; end
                    2'd2: begin pass_is_radix2 = 1'b0; blocks_max = 6'd15; bfs_max = 6'd3;  end
                    2'd3: begin pass_is_radix2 = 1'b1; blocks_max = 6'd63; bfs_max = 6'd1;  end
                    default: begin pass_is_radix2 = 1'b0; blocks_max = 6'd0; bfs_max = 6'd0; end
                endcase
            end

            PE_MODE_INTT: begin
                pipe_lat     = 3'd4;
                pass_uses_tf = 1'b1;
                last_pass    = (pass_idx_r == 2'd3);
                unique case (pass_idx_r)
                    2'd0: begin pass_is_radix2 = 1'b1; blocks_max = 6'd63; bfs_max = 6'd1;  end
                    2'd1: begin pass_is_radix2 = 1'b0; blocks_max = 6'd15; bfs_max = 6'd3;  end
                    2'd2: begin pass_is_radix2 = 1'b0; blocks_max = 6'd3;  bfs_max = 6'd15; end
                    2'd3: begin pass_is_radix2 = 1'b0; blocks_max = 6'd0;  bfs_max = 6'd63; end
                    default: begin pass_is_radix2 = 1'b0; blocks_max = 6'd0; bfs_max = 6'd0; end
                endcase
            end

            PE_MODE_CWM: begin
                // One pass, 64 blocks x 1 issue
                pass_is_radix2 = 1'b0;
                blocks_max     = 6'd63;
                bfs_max        = 6'd0;
                pipe_lat       = 3'd4;
                pass_uses_tf   = 1'b1;
                last_pass      = 1'b1;
            end

            PE_MODE_COMP,
            PE_MODE_DECOMP: begin
                // Simple single-pass stream placeholder
                pass_is_radix2 = 1'b0;
                blocks_max     = 6'd63;
                bfs_max        = 6'd0;
                pipe_lat       = 3'd3;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end

            PE_MODE_ADDSUB: begin
                // Simple single-pass stream placeholder
                pass_is_radix2 = 1'b0;
                blocks_max     = 6'd63;
                bfs_max        = 6'd0;
                pipe_lat       = 3'd1;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end

            default: begin
                pass_is_radix2 = 1'b0;
                blocks_max     = 6'd0;
                bfs_max        = 6'd0;
                pipe_lat       = 3'd1;
                pass_uses_tf   = 1'b0;
                last_pass      = 1'b1;
            end
        endcase
    end

    assign bf_last    = (bf_cnt_r    == bfs_max);
    assign block_last = (block_cnt_r == blocks_max);
    assign issue_last = bf_last && block_last;

    // =========================================================================
    // PE drain detection
    // =========================================================================
    assign all_pe_idle =
        !pe0_valid_i &&
        !pe1_valid_i &&
        !pe2_valid_i &&
        !pe2_valid_m_i &&
        !pe3_valid_i;

    assign drain_done = all_pe_idle || (drain_cnt_r == 3'd0);

    // =========================================================================
    // Next-state logic
    // =========================================================================
    always_comb begin
        state_n      = state_r;
        pass_idx_n   = pass_idx_r;
        block_cnt_n  = block_cnt_r;
        bf_cnt_n     = bf_cnt_r;
        drain_cnt_n  = drain_cnt_r;
        issue_addr_n = issue_addr_r;

        unique case (state_r)

            S_IDLE: begin
                if (start_i)
                    state_n = S_SETUP;
            end

            S_SETUP: begin
                // Begin this pass next cycle
                state_n = S_RUN;
            end

            S_RUN: begin
                // one issue per cycle
                issue_addr_n = issue_addr_r + 8'd1;

                if (bf_last) begin
                    bf_cnt_n = 6'd0;

                    if (block_last) begin
                        block_cnt_n = 6'd0;
                        drain_cnt_n = pipe_lat;
                        state_n     = S_DRAIN;
                    end else begin
                        block_cnt_n = block_cnt_r + 6'd1;
                    end
                end else begin
                    bf_cnt_n = bf_cnt_r + 6'd1;
                end
            end

            S_DRAIN: begin
                if (!drain_done) begin
                    drain_cnt_n = drain_cnt_r - 3'd1;
                end else begin
                    state_n = S_NEXT_PASS;
                end
            end

            S_NEXT_PASS: begin
                if (last_pass) begin
                    state_n = S_DONE;
                end else begin
                    pass_idx_n = pass_idx_r + 2'd1;
                    state_n    = S_SETUP;
                end
            end

            S_DONE: begin
                state_n = S_IDLE;
            end

            default: state_n = S_IDLE;
        endcase
    end

    // =========================================================================
    // Sequential state/counter registers
    // =========================================================================
    always_ff @(posedge clk) begin
        if (rst) begin
            state_r      <= S_IDLE;
            op_r         <= PE_MODE_NTT;
            pass_idx_r   <= 2'd0;
            block_cnt_r  <= 6'd0;
            bf_cnt_r     <= 6'd0;
            drain_cnt_r  <= 3'd0;
            issue_addr_r <= 8'd0;

            for (i = 0; i < 4; i = i + 1) begin
                raddr_pipe[i] <= '0;
                wen_pipe[i]   <= 1'b0;
            end
        end else begin
            state_r      <= state_n;
            pass_idx_r   <= pass_idx_n;
            block_cnt_r  <= block_cnt_n;
            bf_cnt_r     <= bf_cnt_n;
            drain_cnt_r  <= drain_cnt_n;
            issue_addr_r <= issue_addr_n;

            // Latch op only when a new job starts
            if (state_r == S_IDLE && start_i) begin
                op_r         <= op_type_i;
                pass_idx_r   <= 2'd0;
                block_cnt_r  <= 6'd0;
                bf_cnt_r     <= 6'd0;
                drain_cnt_r  <= 3'd0;
                issue_addr_r <= 8'd0;
            end

            // Delay line for writeback alignment
            // stage 0 captures the current read issue
            raddr_pipe[0] <= issue_addr_r;
            wen_pipe[0]   <= (state_r == S_RUN);

            // shift remaining stages
            for (i = 1; i < 4; i = i + 1) begin
                raddr_pipe[i] <= raddr_pipe[i-1];
                wen_pipe[i]   <= wen_pipe[i-1];
            end
        end
    end

    // =========================================================================
    // Outputs
    // =========================================================================
    assign ready_o      = (state_r == S_IDLE);
    assign done_o       = (state_r == S_DONE);

    assign pe_ctrl_o    = op_r;
    assign pe_valid_o   = (state_r == S_RUN);

    assign tf_start_o   = (state_r == S_SETUP) && pass_uses_tf;
    assign pass_idx_o   = pass_idx_r;
    assign tf_is_intt_o = (op_r == PE_MODE_INTT);

    assign mem_read_en_o  = (state_r == S_RUN);

    always_comb begin
        mem_write_en_o = 1'b0;
        wAddr_o        = '0;

        unique case (pipe_lat)
            3'd1: begin
                mem_write_en_o = wen_pipe[0];
                wAddr_o        = raddr_pipe[0];
            end
            3'd3: begin
                mem_write_en_o = wen_pipe[2];
                wAddr_o        = raddr_pipe[2];
            end
            default: begin
                mem_write_en_o = wen_pipe[3];
                wAddr_o        = raddr_pipe[3];
            end
        endcase
    end

    assign rAddr_o      = issue_addr_r;
    assign block_cnt_o  = block_cnt_r;
    assign bf_cnt_o     = bf_cnt_r;

endmodule