module memory_controller #(
    parameter DATA_WIDTH = 8,
    parameter ADDR_WIDTH = 16,
    parameter MEM_DEPTH = 65536    // 2^16 bytes = 64KB
)(
    input wire clk,
    input wire rst_n,
    
    // Command interface
    input wire cmd_valid,
    input wire [1:0] cmd_type,     // 0: NOP, 1: READ, 2: WRITE, 3: RESERVED
    input wire [ADDR_WIDTH-1:0] cmd_addr,
    input wire [DATA_WIDTH-1:0] cmd_data,
    output reg cmd_ready,
    
    // Response interface
    output reg rsp_valid,
    output reg [DATA_WIDTH-1:0] rsp_data,
    input wire rsp_ready,
    
    // Memory interface (could be to SDRAM, on-chip memory, etc.)
    output reg mem_we,
    output reg [ADDR_WIDTH-1:0] mem_addr,
    output reg [DATA_WIDTH-1:0] mem_wdata,
    input wire [DATA_WIDTH-1:0] mem_rdata
);

    // Command type definitions
    localparam CMD_NOP = 2'b00;
    localparam CMD_READ = 2'b01;
    localparam CMD_WRITE = 2'b10;
    
    // State machine states
    localparam IDLE = 2'b00;
    localparam READ_STATE = 2'b01;
    localparam WRITE_STATE = 2'b10;
    localparam RESPONSE = 2'b11;
    
    reg [1:0] state;
    reg [DATA_WIDTH-1:0] read_data;
    
    // State machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            cmd_ready <= 1;
            rsp_valid <= 0;
            rsp_data <= 0;
            mem_we <= 0;
            mem_addr <= 0;
            mem_wdata <= 0;
            read_data <= 0;
        end else begin
            case (state)
                IDLE: begin
                    // Default state
                    mem_we <= 0;
                    rsp_valid <= 0;
                    
                    // Process incoming commands
                    if (cmd_valid && cmd_ready) begin
                        cmd_ready <= 0;  // Not ready for new commands
                        
                        case (cmd_type)
                            CMD_READ: begin
                                mem_addr <= cmd_addr;
                                state <= READ_STATE;
                            end
                            
                            CMD_WRITE: begin
                                mem_addr <= cmd_addr;
                                mem_wdata <= cmd_data;
                                mem_we <= 1;
                                state <= WRITE_STATE;
                            end
                            
                            default: begin
                                // NOP or invalid command, stay in IDLE
                                cmd_ready <= 1;
                            end
                        endcase
                    end else begin
                        cmd_ready <= 1;  // Ready for new commands
                    end
                end
                
                READ_STATE: begin
                    // Read operation takes one cycle
                    // In the next cycle, mem_rdata will have the data
                    read_data <= mem_rdata;
                    state <= RESPONSE;
                end
                
                WRITE_STATE: begin
                    // Write operation completes in one cycle
                    mem_we <= 0;
                    state <= IDLE;
                    cmd_ready <= 1;  // Ready for new commands
                end
                
                RESPONSE: begin
                    // Send read response
                    rsp_valid <= 1;
                    rsp_data <= read_data;
                    
                    // Wait for response to be accepted
                    if (rsp_ready) begin
                        rsp_valid <= 0;
                        state <= IDLE;
                        cmd_ready <= 1;  // Ready for new commands
                    end
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule