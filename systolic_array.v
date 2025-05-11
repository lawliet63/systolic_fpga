module systolic_array #(
    parameter ARRAY_SIZE = 3,
    parameter DATA_WIDTH = 8
)(
    input wire clk,
    input wire rst_n,
    input wire start,
    
    // Input data ports
    input wire [DATA_WIDTH-1:0] a_in [0:ARRAY_SIZE-1],  // Input matrix A
    input wire [DATA_WIDTH-1:0] b_in [0:ARRAY_SIZE-1],  // Input matrix B
    input wire valid_in,
    
    // Output data ports
    output wire [2*DATA_WIDTH-1:0] c_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1],  // Output matrix C
    output wire valid_out
);

    // Internal wires for connecting PEs
    wire [DATA_WIDTH-1:0] a_wires [0:ARRAY_SIZE][0:ARRAY_SIZE-1];
    wire [DATA_WIDTH-1:0] b_wires [0:ARRAY_SIZE-1][0:ARRAY_SIZE];
    wire valid_wires [0:ARRAY_SIZE][0:ARRAY_SIZE];
    
    genvar i, j;
    
    // Connect inputs to first row and column of wires
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin: input_connections
            assign a_wires[i][0] = a_in[i];
            assign b_wires[0][i] = b_in[i];
        end
    endgenerate
    
    // Generate the systolic array
    generate
        for (i = 0; i < ARRAY_SIZE; i = i + 1) begin: rows
            for (j = 0; j < ARRAY_SIZE; j = j + 1) begin: cols
                processing_element #(
                    .DATA_WIDTH(DATA_WIDTH)
                ) pe_inst (
                    .clk(clk),
                    .rst_n(rst_n),
                    .a_in(a_wires[i][j]),
                    .b_in(b_wires[j][i]),
                    .valid_in(valid_in),
                    
                    .a_out(a_wires[i][j+1]),
                    .b_out(b_wires[j+1][i]),
                    .c_out(c_out[i][j]),
                    .valid_out(valid_wires[i+1][j+1])
                );
            end
        end
    endgenerate
    
    // Connect valid_in to the top-left PE
    assign valid_wires[0][0] = valid_in;
    
    // Output valid signal comes from the bottom-right PE
    assign valid_out = valid_wires[ARRAY_SIZE][ARRAY_SIZE-1];

endmodule