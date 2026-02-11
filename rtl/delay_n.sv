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
            // Shift logic:
            // Input goes into index 0.
            // Old data shifts "up" the array.
            // Output comes from the top (DEPTH-1).
            shift_reg <= {shift_reg[DEPTH-2:0], data_i};
        end
    end

    assign data_o = shift_reg[DEPTH-1];

endmodule
