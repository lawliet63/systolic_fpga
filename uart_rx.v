module uart_rx #(
    parameter CLK_FREQ = 50000000,  // 50 MHz
    parameter BAUD_RATE = 115200    // 115200 bps
)(
    input wire clk,                 // System clock
    input wire rst_n,               // Active low reset
    input wire rx,                  // UART RX signal
    output reg [7:0] data_out,      // Received data
    output reg data_valid           // Data valid signal
);

    // Calculate clock ticks per bit
    localparam TICKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    
    // States for the UART RX state machine
    localparam IDLE = 3'd0;
    localparam START_BIT = 3'd1;
    localparam DATA_BITS = 3'd2;
    localparam STOP_BIT = 3'd3;
    localparam CLEANUP = 3'd4;
    
    // Registers
    reg [2:0] state;
    reg [15:0] bit_timer;
    reg [2:0] bit_counter;
    reg [7:0] rx_data;
    
    // Synchronize RX input with clock to prevent metastability
    reg rx_sync_0;
    reg rx_sync_1;
    
    always @(posedge clk) begin
        rx_sync_0 <= rx;
        rx_sync_1 <= rx_sync_0;
    end
    
    // UART RX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_timer <= 0;
            bit_counter <= 0;
            rx_data <= 0;
            data_out <= 0;
            data_valid <= 0;
        end else begin
            case (state)
                IDLE: begin
                    data_valid <= 0;
                    bit_timer <= 0;
                    bit_counter <= 0;
                    
                    // Wait for start bit (falling edge on RX)
                    if (rx_sync_1 == 0) begin
                        state <= START_BIT;
                    end
                end
                
                START_BIT: begin
                    // Sample in the middle of the start bit
                    if (bit_timer == TICKS_PER_BIT/2) begin
                        // Verify we're still in the start bit
                        if (rx_sync_1 == 0) begin
                            bit_timer <= 0;
                            state <= DATA_BITS;
                        end else begin
                            // False start, go back to idle
                            state <= IDLE;
                        end
                    end else begin
                        bit_timer <= bit_timer + 1;
                    end
                end
                
                DATA_BITS: begin
                    // Sample in the middle of each data bit
                    if (bit_timer == TICKS_PER_BIT) begin
                        bit_timer <= 0;
                        
                        // Shift in the received bit
                        rx_data <= {rx_sync_1, rx_data[7:1]};
                        
                        // Increment bit counter
                        if (bit_counter < 7) begin
                            bit_counter <= bit_counter + 1;
                        end else begin
                            // We have received all 8 bits
                            bit_counter <= 0;
                            state <= STOP_BIT;
                        end
                    end else begin
                        bit_timer <= bit_timer + 1;
                    end
                end
                
                STOP_BIT: begin
                    // Wait for stop bit to complete
                    if (bit_timer == TICKS_PER_BIT) begin
                        // Check if stop bit is high
                        if (rx_sync_1 == 1) begin
                            data_out <= rx_data;
                            data_valid <= 1;
                        end
                        bit_timer <= 0;
                        state <= CLEANUP;
                    end else begin
                        bit_timer <= bit_timer + 1;
                    end
                end
                
                CLEANUP: begin
                    // Extra clock cycle to clear data_valid
                    data_valid <= 0;
                    state <= IDLE;
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule