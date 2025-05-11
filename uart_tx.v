module uart_tx #(
    parameter CLK_FREQ = 50000000,  // 50 MHz
    parameter BAUD_RATE = 115200    // 115200 bps
)(
    input wire clk,                 // System clock
    input wire rst_n,               // Active low reset
    input wire [7:0] data_in,       // Data to transmit
    input wire data_valid,          // Input data valid signal
    output reg tx,                  // UART TX signal
    output reg busy                 // Transmitter busy signal
);

    // Calculate clock ticks per bit
    localparam TICKS_PER_BIT = CLK_FREQ / BAUD_RATE;
    
    // States for the UART TX state machine
    localparam IDLE = 3'd0;
    localparam START_BIT = 3'd1;
    localparam DATA_BITS = 3'd2;
    localparam STOP_BIT = 3'd3;
    
    // Registers
    reg [2:0] state;
    reg [15:0] bit_timer;
    reg [2:0] bit_counter;
    reg [7:0] tx_data;
    
    // UART TX state machine
    always @(posedge clk or negedge rst_n) begin
        if (!rst_n) begin
            state <= IDLE;
            bit_timer <= 0;
            bit_counter <= 0;
            tx_data <= 0;
            tx <= 1;          // Idle state is high
            busy <= 0;
        end else begin
            case (state)
                IDLE: begin
                    tx <= 1;      // Idle state is high
                    busy <= 0;
                    bit_timer <= 0;
                    bit_counter <= 0;
                    
                    // Start transmission when data_valid is asserted
                    if (data_valid) begin
                        tx_data <= data_in;
                        busy <= 1;
                        state <= START_BIT;
                    end
                end
                
                START_BIT: begin
                    // Send start bit (low)
                    tx <= 0;
                    
                    // Wait for one bit time
                    if (bit_timer < TICKS_PER_BIT-1) begin
                        bit_timer <= bit_timer + 1;
                    end else begin
                        bit_timer <= 0;
                        state <= DATA_BITS;
                    end
                end
                
                DATA_BITS: begin
                    // Send data bits LSB first
                    tx <= tx_data[bit_counter];
                    
                    // Wait for one bit time
                    if (bit_timer < TICKS_PER_BIT-1) begin
                        bit_timer <= bit_timer + 1;
                    end else begin
                        bit_timer <= 0;
                        
                        // Increment bit counter
                        if (bit_counter < 7) begin
                            bit_counter <= bit_counter + 1;
                        end else begin
                            // We have sent all 8 bits
                            bit_counter <= 0;
                            state <= STOP_BIT;
                        end
                    end
                end
                
                STOP_BIT: begin
                    // Send stop bit (high)
                    tx <= 1;
                    
                    // Wait for one bit time
                    if (bit_timer < TICKS_PER_BIT-1) begin
                        bit_timer <= bit_timer + 1;
                    end else begin
                        bit_timer <= 0;
                        state <= IDLE;
                    end
                end
                
                default: begin
                    state <= IDLE;
                end
            endcase
        end
    end

endmodule