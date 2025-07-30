module baud_gen #(parameter CLK_FREQ = 50000000, parameter BAUD_RATE = 9600)(
    input wire clk,
    input wire reset,
    output reg baud_tick
);

    localparam integer DIVISOR = CLK_FREQ / (BAUD_RATE * 16);
    reg [$clog2(DIVISOR)-1:0] counter = 0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            counter <= 0;
            baud_tick <= 0;
        end else if (counter == DIVISOR - 1) begin
            counter <= 0;
            baud_tick <= 1;
        end else begin
            counter <= counter + 1;
            baud_tick <= 0;
        end
    end

endmodule
module fifo #(
    parameter DATA_WIDTH = 8,
    parameter DEPTH = 16
)(
    input wire clk,
    input wire reset,

    input wire wr_en,
    input wire [DATA_WIDTH-1:0] wr_data,

    input wire rd_en,
    output reg [DATA_WIDTH-1:0] rd_data,

    output wire empty,
    output wire full
);
    reg [DATA_WIDTH-1:0] mem [0:DEPTH-1];
    reg [$clog2(DEPTH):0] rd_ptr = 0;
    reg [$clog2(DEPTH):0] wr_ptr = 0;
    reg [$clog2(DEPTH)+1:0] count = 0;

    assign empty = (count == 0);
    assign full = (count == DEPTH);

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            rd_ptr <= 0;
            wr_ptr <= 0;
            count <= 0;
        end else begin
            if (wr_en && !full) begin
                mem[wr_ptr] <= wr_data;
                wr_ptr <= wr_ptr + 1;
                count <= count + 1;
            end

            if (rd_en && !empty) begin
                rd_data <= mem[rd_ptr];
                rd_ptr <= rd_ptr + 1;
                count <= count - 1;
            end
        end
    end
endmodule
module uart_fifo_top (
    input  wire clk,
    input  wire reset,

    // TX interface
    input  wire [7:0] tx_data,
    input  wire tx_write,
    output wire tx_full,
    output wire tx,

    // RX interface
    input  wire rx,
    output wire [7:0] rx_data,
    input  wire rx_read,
    output wire rx_empty
);
    wire baud_tick;

    baud_gen #(
        .CLK_FREQ(50000000),
        .BAUD_RATE(9600)
    ) baud_inst (
        .clk(clk),
        .reset(reset),
        .baud_tick(baud_tick)
    );

    uart_tx_fifo tx_inst (
        .clk(clk),
        .reset(reset),
        .baud_tick(baud_tick),
        .tx_data_in(tx_data),
        .tx_write(tx_write),
        .tx_full(tx_full),
        .tx(tx),
        .tx_busy()
    );

    uart_rx_fifo rx_inst (
        .clk(clk),
        .reset(reset),
        .baud_tick(baud_tick),
        .rx(rx),
        .rx_data_out(rx_data),
        .rx_read(rx_read),
        .rx_empty(rx_empty)
    );
endmodule
module uart_receiver (
    input wire clk,             // System clock
    input wire reset,           // Active high reset
    input wire rx,              // UART RX line
    input wire baud_tick,       // Tick from baud rate generator (16× baud rate)

    output reg [7:0] data_out,  // Received byte
    output reg data_ready       // Goes high when byte is received
);

    // State encoding
    localparam IDLE  = 3'b000;
    localparam START = 3'b001;
    localparam DATA  = 3'b010;
    localparam STOP  = 3'b011;
    localparam DONE  = 3'b100;

    reg [2:0] state = IDLE;

    reg [3:0] tick_counter = 0;  // Counts 0 to 15
    reg [2:0] bit_index = 0;     // Counts 0 to 7 for 8 bits

    reg [7:0] shift_reg = 0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state        <= IDLE;
            data_ready   <= 0;
            tick_counter <= 0;
            bit_index    <= 0;
            shift_reg    <= 0;
        end else if (baud_tick) begin
            case (state)
                // Wait for falling edge (start bit)
                IDLE: begin
                    data_ready <= 0;
                    if (~rx) begin  // Start bit detected (rx is LOW)
                        tick_counter <= 0;
                        state <= START;
                    end
                end

                // Wait for middle of start bit (tick 8)
                START: begin
                    if (tick_counter == 7) begin
                        if (~rx) begin  // Confirm it's still low
                            tick_counter <= 0;
                            bit_index <= 0;
                            state <= DATA;
                        end else begin
                            state <= IDLE;  // False start bit
                        end
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                // Receive 8 data bits
                DATA: begin
                    if (tick_counter == 15) begin
                        tick_counter <= 0;
                        shift_reg <= {rx, shift_reg[7:1]};  // Shift in LSB first
                        if (bit_index == 7) begin
                            state <= STOP;
                        end else begin
                            bit_index <= bit_index + 1;
                        end
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                // Check stop bit
                STOP: begin
                    if (tick_counter == 15) begin
                        data_out <= shift_reg;
                        data_ready <= rx;  // Only assert if stop bit is HIGH
                        state <= IDLE;
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
module uart_rx_fifo (
    input  wire clk,
    input  wire reset,
    input  wire baud_tick,
    input  wire rx,
    
    output wire [7:0] rx_data_out,
    input  wire rx_read,
    output wire rx_empty
);
    wire [7:0] rx_data_internal;
    wire rx_data_ready;

    // RX core
    uart_receiver rx_core (
        .clk(clk),
        .reset(reset),
        .rx(rx),
        .baud_tick(baud_tick),
        .data_out(rx_data_internal),
        .data_ready(rx_data_ready)
    );

    // RX FIFO
    fifo #(.DATA_WIDTH(8), .DEPTH(16)) rx_fifo (
        .clk(clk),
        .reset(reset),
        .wr_en(rx_data_ready),
        .wr_data(rx_data_internal),
        .rd_en(rx_read),
        .rd_data(rx_data_out),
        .empty(rx_empty),
        .full() // ignored
    );
endmodule
module uart_transmitter (
    input wire clk,
    input wire reset,
    input wire tx_start,             // Signal to start transmission
    input wire [7:0] tx_data,        // Data byte to transmit
    input wire baud_tick,            // Tick from baud rate generator (16× baud rate)

    output reg tx,                   // UART TX line
    output reg tx_busy,              // HIGH when transmitting
    output reg tx_done               // HIGH when done (1 clk cycle)
);

    // State encoding
    localparam IDLE  = 3'b000;
    localparam START = 3'b001;
    localparam DATA  = 3'b010;
    localparam STOP  = 3'b011;
    localparam DONE  = 3'b100;

    reg [2:0] state = IDLE;

    reg [3:0] tick_counter = 0;      // Counts up to 15 (16 ticks per bit)
    reg [2:0] bit_index = 0;         // 0 to 7 for 8 bits

    reg [7:0] shift_reg = 0;

    always @(posedge clk or posedge reset) begin
        if (reset) begin
            state        <= IDLE;
            tx           <= 1'b1;   // TX line idle is HIGH
            tx_busy      <= 0;
            tx_done      <= 0;
            tick_counter <= 0;
            bit_index    <= 0;
        end else if (baud_tick) begin
            case (state)
                IDLE: begin
                    tx_done <= 0;
                    if (tx_start) begin
                        tx_busy      <= 1;
                        shift_reg    <= tx_data;
                        tick_counter <= 0;
                        state        <= START;
                        tx           <= 0; // Start bit
                    end
                end

                START: begin
                    if (tick_counter == 15) begin
                        tick_counter <= 0;
                        bit_index <= 0;
                        state <= DATA;
                        tx <= shift_reg[0]; // First data bit
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                DATA: begin
                    if (tick_counter == 15) begin
                        tick_counter <= 0;
                        bit_index <= bit_index + 1;
                        if (bit_index == 7) begin
                            state <= STOP;
                            tx <= 1; // Stop bit
                        end else begin
                            shift_reg <= shift_reg >> 1;
                            tx <= shift_reg[1]; // Next data bit
                        end
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                STOP: begin
                    if (tick_counter == 15) begin
                        state <= DONE;
                        tick_counter <= 0;
                        tx_done <= 1;
                        tx_busy <= 0;
                    end else begin
                        tick_counter <= tick_counter + 1;
                    end
                end

                DONE: begin
                    state <= IDLE;
                    tx_done <= 0;
                end

                default: state <= IDLE;
            endcase
        end
    end

endmodule
module uart_tx_fifo (
    input  wire clk,
    input  wire reset,
    input  wire baud_tick,
    input  wire [7:0] tx_data_in,
    input  wire tx_write,
    output wire tx_full,

    output wire tx,
    output wire tx_busy
);
    wire fifo_empty;
    wire [7:0] fifo_data_out;
    reg tx_start = 0;
    wire tx_done;

    // FIFO for TX buffer
    fifo #(.DATA_WIDTH(8), .DEPTH(16)) tx_fifo (
        .clk(clk),
        .reset(reset),
        .wr_en(tx_write),
        .wr_data(tx_data_in),
        .rd_en(tx_done),
        .rd_data(fifo_data_out),
        .empty(fifo_empty),
        .full(tx_full)
    );

    // Transmitter logic
    uart_transmitter tx_core (
        .clk(clk),
        .reset(reset),
        .tx_start(tx_start),
        .tx_data(fifo_data_out),
        .baud_tick(baud_tick),
        .tx(tx),
        .tx_busy(tx_busy),
        .tx_done(tx_done)
    );

    // Start TX if not busy and FIFO is not empty
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            tx_start <= 0;
        end else begin
            tx_start <= (~fifo_empty && ~tx_busy);
        end
    end
endmodule
