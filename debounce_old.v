module debounce #
(
    parameter integer RISE_STABLE = 10000,   // cycles required for low→high
    parameter integer FALL_STABLE = 10000    // cycles required for high→low
)
(
    input  wire clk,          // 100 MHz clock
    input  wire rst,          // synchronous reset
    input  wire sig_in,       // noisy input
    output reg  sig_out       // debounced output
);

    reg        prev_in;
    reg [15:0] counter;

    always @(posedge clk) begin
        if (rst) begin
            sig_out <= 1'b0;
            prev_in <= 1'b0;
            counter <= 16'd0;
        end else begin

            // If input changed -> reset counter
            if (sig_in != prev_in) begin
                prev_in <= sig_in;
                counter <= 16'd0;
            end else begin
                // Count stable cycles
                if (counter < 16'hFFFF)
                    counter <= counter + 16'd1;

                // Decide when to update output
                if (sig_out == 1'b0 && prev_in == 1'b1) begin
                    // LOW → HIGH transition
                    if (counter >= RISE_STABLE)
                        sig_out <= 1'b1;
                end
                else if (sig_out == 1'b1 && prev_in == 1'b0) begin
                    // HIGH → LOW transition
                    if (counter >= FALL_STABLE)
                        sig_out <= 1'b0;
                end
            end

        end
    end

endmodule
