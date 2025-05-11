module processing_element #(
    parameter DATA_WIDTH = 8
)(
    input wire clk,
    input wire rst_n,
    input wire [DATA_WIDTH-1:0] a_in,    // Input from left
    input wire [DATA_WIDTH-1:0] b_in,    // Input from top
    input wire valid_in,                 // Input data valid signal
    
    output reg [DATA_WIDTH-1:0] a_out,   // Output to right
    output reg [DATA_WIDTH-1:0] b_out,   // Output to bottom
    output reg [2*DATA_WIDTH-1:0] c_out, // Accumulated result
    output reg valid_out               // Output data valid signal
);

    // Internal registers
    reg [DATA_WIDTH-1:0] a_reg;
    reg [DATA_WIDTH-1:0] b_reg;
    reg [2*DATA_WIDTH-1:0] c_reg;
    reg valid_reg;

    // Combinational logic for calculation
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            // Reset all registers
            a_reg <= 0;
            b_reg <= 0;
            c_reg <= 0;
            valid_reg <= 0;
            
            a_out <= 0;
            b_out <= 0;
            c_out <= 0;
            valid_out <= 0;
        end else begin
            // Register inputs
            a_reg <= a_in;
            b_reg <= b_in;
            valid_reg <= valid_in;
            
            // Pass data to outputs
            a_out <= a_reg;
            b_out <= b_reg;
            valid_out <= valid_reg;
            
            // Multiply and accumulate
            if (valid_in) begin
                c_reg <= c_reg + (a_in * b_in);
            end
            
            // Output accumulated result
            c_out <= c_reg;
        end
    end

endmodule