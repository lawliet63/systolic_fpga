module top_module #(
    parameter ARRAY_SIZE = 3,
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 16,
    parameter MEM_DEPTH = 65536,
    parameter CLK_FREQ = 50000000,  // 50 MHz
    parameter BAUD_RATE = 115200    // 115200 bps
)(
    // Clock and reset
    input wire CLOCK_50,        // 50 MHz clock from DE0-Nano
    input wire [1:0] KEY,       // KEY[0] used as reset (active low)
    
    // UART interface
    input wire UART_RXD,        // UART receive line
    output wire UART_TXD,       // UART transmit line
    
    // LED indicators for status
    output reg [7:0] LED,       // 8 LEDs on DE0-Nano
    
    // GPIO for debugging (optional)
    output reg [35:0] GPIO_0,   // GPIO port 0
    output reg [35:0] GPIO_1    // GPIO port 1
);

    // Internal signals
    wire clk;
    wire rst_n;
    
    // Use PLL to generate stable clock
    pll pll_inst (
        .inclk0(CLOCK_50),
        .c0(clk)
    );
    
    // Reset signal
    assign rst_n = KEY[0];
    
    // UART signals
    wire [7:0] uart_rx_data;
    wire uart_rx_valid;
    reg [7:0] uart_tx_data;
    reg uart_tx_valid;
    wire uart_tx_busy;
    
    // Memory controller signals
    reg mem_cmd_valid;
    reg [1:0] mem_cmd_type;
    reg [ADDR_WIDTH-1:0] mem_cmd_addr;
    reg [DATA_WIDTH-1:0] mem_cmd_data;
    wire mem_cmd_ready;
    wire mem_rsp_valid;
    wire [DATA_WIDTH-1:0] mem_rsp_data;
    reg mem_rsp_ready;
    
    // Memory signals
    wire mem_we;
    wire [ADDR_WIDTH-1:0] mem_addr;
    wire [DATA_WIDTH-1:0] mem_wdata;
    wire [DATA_WIDTH-1:0] mem_rdata;
    
    // Systolic array signals
    reg [DATA_WIDTH-1:0] a_in [0:ARRAY_SIZE-1];
    reg [DATA_WIDTH-1:0] b_in [0:ARRAY_SIZE-1];
    wire [2*DATA_WIDTH-1:0] c_out [0:ARRAY_SIZE-1][0:ARRAY_SIZE-1];
    reg systolic_start;
    reg systolic_valid_in;
    wire systolic_valid_out;
    
    // Protocol command definitions
    localparam CMD_NOOP     = 8'h00;
    localparam CMD_WRITE_A  = 8'h01;
    localparam CMD_WRITE_B  = 8'h02;
    localparam CMD_READ_C   = 8'h03;
    localparam CMD_COMPUTE  = 8'h04;
    localparam CMD_STATUS   = 8'h05;
    
    // Status codes
    localparam STATUS_IDLE     = 8'h00;
    localparam STATUS_BUSY     = 8'h01;
    localparam STATUS_DONE     = 8'h02;
    localparam STATUS_ERROR    = 8'hFF;
    
    // Protocol state machine states
    localparam PROTO_IDLE          = 4'd0;
    localparam PROTO_CMD           = 4'd1;
    localparam PROTO_ADDR_HIGH     = 4'd2;
    localparam PROTO_ADDR_LOW      = 4'd3;
    localparam PROTO_DATA          = 4'd4;
    localparam PROTO_EXECUTE       = 4'd5;
    localparam PROTO_RESPONSE      = 4'd6;
    localparam PROTO_COMPUTE       = 4'd7;
    localparam PROTO_READ_RESULT   = 4'd8;
    
    reg [3:0] proto_state;
    reg [7:0] protocol_cmd;
    reg [15:0] protocol_addr;
    reg [7:0] protocol_data;
    reg [7:0] status_reg;
    
    // Instantiate UART modules
    uart_rx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_rx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .rx(UART_RXD),
        .data_out(uart_rx_data),
        .data_valid(uart_rx_valid)
    );
    
    uart_tx #(
        .CLK_FREQ(CLK_FREQ),
        .BAUD_RATE(BAUD_RATE)
    ) uart_tx_inst (
        .clk(clk),
        .rst_n(rst_n),
        .data_in(uart_tx_data),
        .data_valid(uart_tx_valid),
        .tx(UART_TXD),
        .busy(uart_tx_busy)
    );
    
    // Instantiate memory controller
    memory_controller #(
        .DATA_WIDTH(DATA_WIDTH),
        .ADDR_WIDTH(ADDR_WIDTH),
        .MEM_DEPTH(MEM_DEPTH)
    ) mem_ctrl_inst (
        .clk(clk),
        .rst_n(rst_n),
        .cmd_valid(mem_cmd_valid),
        .cmd_type(mem_cmd_type),
        .cmd_addr(mem_cmd_addr),
        .cmd_data(mem_cmd_data),
        .cmd_ready(mem_cmd_ready),
        .rsp_valid(mem_rsp_valid),
        .rsp_data(mem_rsp_data),
        .rsp_ready(mem_rsp_ready),
        .mem_we(mem_we),
        .mem_addr(mem_addr),
        .mem_wdata(mem_wdata),
        .mem_rdata(mem_rdata)
    );
    
    // Instantiate systolic array
    systolic_array #(
        .ARRAY_SIZE(ARRAY_SIZE),
        .DATA_WIDTH(DATA_WIDTH)
    ) systolic_array_inst (
        .clk(clk),
        .rst_n(rst_n),
        .start(systolic_start),
        .a_in(a_in),
        .b_in(b_in),
        .valid_in(systolic_valid_in),
        .c_out(c_out),
        .valid_out(systolic_valid_out)
    );
    
    // On-chip memory for matrices
    // Matrix A, B, and C storage
    reg [DATA_WIDTH-1:0] mem [0:MEM_DEPTH-1];
    
    // Memory read path
    assign mem_rdata = mem[mem_addr];
    
    // Memory write path
    always @(posedge clk) begin
        if (mem_we) begin
            mem[mem_addr] <= mem_wdata;
        end
    end
    
    // Protocol state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            proto_state <= PROTO_IDLE;
            protocol_cmd <= CMD_NOOP;
            protocol_addr <= 0;
            protocol_data <= 0;
            mem_cmd_valid <= 0;
            mem_cmd_type <= 0;
            mem_cmd_addr <= 0;
            mem_cmd_data <= 0;
            mem_rsp_ready <= 0;
            uart_tx_valid <= 0;
            uart_tx_data <= 0;
            systolic_start <= 0;
            systolic_valid_in <= 0;
            status_reg <= STATUS_IDLE;
            LED <= 8'h00;
        end else begin
            // Default values
            mem_cmd_valid <= 0;
            uart_tx_valid <= 0;
            mem_rsp_ready <= 0;
            systolic_start <= 0;
            
            // Update LEDs with status
            LED <= status_reg;
            
            case (proto_state)
                PROTO_IDLE: begin
                    if (uart_rx_valid) begin
                        protocol_cmd <= uart_rx_data;
                        proto_state <= PROTO_CMD;
                    end
                end
                
                PROTO_CMD: begin
                    case (protocol_cmd)
                        CMD_NOOP: begin
                            // No operation, send ACK
                            uart_tx_data <= CMD_NOOP;
                            uart_tx_valid <= 1;
                            proto_state <= PROTO_IDLE;
                        end
                        
                        CMD_WRITE_A, CMD_WRITE_B: begin
                            // For write commands, we need address and data
                            if (uart_rx_valid) begin
                                protocol_addr[15:8] <= uart_rx_data;
                                proto_state <= PROTO_ADDR_HIGH;
                            end
                        end
                        
                        CMD_READ_C: begin
                            // For read command, we need address
                            if (uart_rx_valid) begin
                                protocol_addr[15:8] <= uart_rx_data;
                                proto_state <= PROTO_ADDR_HIGH;
                            end
                        end
                        
                        CMD_COMPUTE: begin
                            // Start computation
                            status_reg <= STATUS_BUSY;
                            systolic_start <= 1;  // Trigger computation
                            systolic_valid_in <= 1;  // Input data is valid
                            proto_state <= PROTO_COMPUTE;
                        end
                        
                        CMD_STATUS: begin
                            // Send status
                            uart_tx_data <= status_reg;
                            uart_tx_valid <= 1;
                            proto_state <= PROTO_IDLE;
                        end
                        
                        default: begin
                            // Unknown command
                            uart_tx_data <= STATUS_ERROR;
                            uart_tx_valid <= 1;
                            proto_state <= PROTO_IDLE;
                        end
                    endcase
                end
                
                PROTO_ADDR_HIGH: begin
                    if (uart_rx_valid) begin
                        protocol_addr[7:0] <= uart_rx_data;
                        proto_state <= PROTO_ADDR_LOW;
                    end
                end
                
                PROTO_ADDR_LOW: begin
                    if (protocol_cmd == CMD_READ_C) begin
                        // For read, execute now
                        mem_cmd_valid <= 1;
                        mem_cmd_type <= 2'b01;  // Read
                        mem_cmd_addr <= protocol_addr;
                        proto_state <= PROTO_EXECUTE;
                    end else if (uart_rx_valid) begin
                        // For write, get data first
                        protocol_data <= uart_rx_data;
                        proto_state <= PROTO_DATA;
                    end
                end
                
                PROTO_DATA: begin
                    // Execute write
                    mem_cmd_valid <= 1;
                    mem_cmd_type <= 2'b10;  // Write
                    mem_cmd_addr <= protocol_addr;
                    mem_cmd_data <= protocol_data;
                    proto_state <= PROTO_EXECUTE;
                end
                
                PROTO_EXECUTE: begin
                    if (mem_cmd_ready) begin
                        if (protocol_cmd == CMD_READ_C) begin
                            // For read, wait for response
                            mem_rsp_ready <= 1;
                            proto_state <= PROTO_RESPONSE;
                        end else {
                            // For write, send ACK
                            uart_tx_data <= protocol_cmd;  // Echo command as ACK
                            uart_tx_valid <= 1;
                            proto_state <= PROTO_IDLE;
                        }
                    end
                end
                
                PROTO_RESPONSE: begin
                    if (mem_rsp_valid && mem_rsp_ready) begin
                        // Send read data
                        uart_tx_data <= mem_rsp_data;
                        uart_tx_valid <= 1;
                        proto_state <= PROTO_IDLE;
                    end
                end
                
                PROTO_COMPUTE: begin
                    // Wait for computation to complete
                    systolic_valid_in <= 0;  // Clear valid_in after first cycle
                    
                    if (systolic_valid_out) begin
                        // Computation done
                        status_reg <= STATUS_DONE;
                        proto_state <= PROTO_IDLE;
                    end
                end
                
                default: begin
                    proto_state <= PROTO_IDLE;
                end
            endcase
        end
    end
    
    // PLL module for clock generation
    // You'll need to configure this in Quartus using IP Catalog
    module pll (
        input inclk0,
        output c0
    );
        // This will be generated by Quartus IP Catalog
    endmodule

endmodule