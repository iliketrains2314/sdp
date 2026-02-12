// Measures overlap high time of (sig_in & mos_state[1])
// 'period' = number of clk cycles where sig_in && mos_state[1] were both high
// 'valid'  = 1 for one clk when 'period' is updated
module bemf_speed_estimator (
    input  wire        clk,        // system clock
    input  wire        rst,        // sync reset, active high
    input  wire        sig_in,     // BEMF comparator output
    input  wire [5:0]  mos_state,
    output reg  [31:0] period,     // measured high-time in clk cycles
    output reg         valid       // 1 clk pulse when 'period' is updated
);

    reg        sig_in_d;   // delayed input for edge detect
    wire       sig_in_fall;
    reg [31:0] counter;    // counts while we're between rise and fall

    // Register previous value of sig_in
    always @(posedge clk) begin
        if (rst)
            sig_in_d <= 1'b0;
        else
            sig_in_d <= sig_in;
    end

    // Falling edge detection on sig_in
    assign sig_in_fall = ~sig_in & sig_in_d;  // 1 -> 0

    // Counter + period latch + valid pulse
    always @(posedge clk) begin
        if (rst) begin
            counter <= 32'd0;
            period  <= 32'd0;
            valid   <= 1'b0;
        end else begin
            // Count only when both are high
            if (sig_in & mos_state[1]) begin
                counter <= counter + 32'd1;
            end

            // When sig_in falls, latch the measurement
            if (sig_in_fall) begin
                period  <= counter;     // measured overlap time
                valid   <= 1'b1;        // pulse valid on every fall
                counter <= 32'd0;       // ready for next pulse
            end else begin
                valid   <= 1'b0;
            end
        end
    end

endmodule
