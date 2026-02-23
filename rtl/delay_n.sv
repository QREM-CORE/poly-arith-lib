/*
 * Module Name: delay_n
 * Author(s): Kiet Le
 * Description: Generic shift register/delay line.
 * Parameters:
 * - DWIDTH: Bit width of the signal (default 12 for coefficients)
 * - DEPTH: Number of clock cycles to delay (default 3)
 */

module delay_n #(
    parameter int DWIDTH = 12,
    parameter int DEPTH = 3
) (
    input  logic                clk,
    input  logic                rst,
    input  logic [DWIDTH-1:0]   data_i,
    output logic [DWIDTH-1:0]   data_o
);

    logic [DEPTH-1:0][DWIDTH-1:0] shift_reg;

    always_ff @(posedge clk) begin
        if (rst) begin
            shift_reg <= '0;
        end else begin
            // Input always goes to index 0
            shift_reg[0] <= data_i;

            // Shift older data up (automatically skipped if DEPTH == 1)
            for (int i = 1; i < DEPTH; i++) begin
                shift_reg[i] <= shift_reg[i-1];
            end
        end
    end

    assign data_o = shift_reg[DEPTH-1];

endmodule
