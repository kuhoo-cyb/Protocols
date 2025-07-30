
`timescale 1ns / 1ps

module uart_fifo_tb;

    reg clk = 0;
    reg reset = 1;

    // TX signals
    reg  [7:0] tx_data;
    reg        tx_write;
    wire       tx_full;

    // RX signals
    wire [7:0] rx_data;
    reg        rx_read;
    wire       rx_empty;

    wire tx;
    wire rx = tx; // Loopback connection

    // Instantiate UART with FIFO
    uart_fifo_top uut (
        .clk(clk),
        .reset(reset),
        .tx_data(tx_data),
        .tx_write(tx_write),
        .tx_full(tx_full),
        .tx(tx),
        .rx(rx),
        .rx_data(rx_data),
        .rx_read(rx_read),
        .rx_empty(rx_empty)
    );

    // Clock generation (50 MHz)
    always #10 clk = ~clk;

    integer i;

    initial begin
        $dumpfile("uart_fifo_tb.vcd");
        $dumpvars(0, uart_fifo_tb);

        // Initial state
        tx_data  = 0;
        tx_write = 0;
        rx_read  = 0;

        // Apply reset
        #100;
        reset = 0;

        // Transmit 10 bytes
        for (i = 0; i < 10; i = i + 1) begin
            @(negedge clk);
            if (!tx_full) begin
                tx_data  = i + 8'h41; // ASCII 'A' + i
                tx_write = 1;
                @(negedge clk);
                tx_write = 0;
            end
            #10000; // wait between writes
        end

        // Wait for all bytes to be received
        #500000;

        // Read all bytes from RX FIFO
        for (i = 0; i < 10; i = i + 1) begin
            if (!rx_empty) begin
                @(negedge clk);
                rx_read = 1;
                @(negedge clk);
                rx_read = 0;
                @(posedge clk);
                $display("Received Byte %0d: %h (%c)", i, rx_data, rx_data);
            end
            #20000;
        end

        #1000;
        $display("Simulation completed.");
        $finish;
    end

endmodule
