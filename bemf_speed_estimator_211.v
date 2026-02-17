// Measures overlap high time of (sig_in & mos_state[1])
// 'period' = number of clk cycles where sig_in && mos_state[1] were both high
// 'valid'  = 1 for one clk when 'period' is updated
module bemf_speed_estimator (
    input  wire        clk,
    input  wire        rst,
    input  wire        sig_in,        // raw comparator output (async)
    input  wire [5:0]  mos_state,     // sector/state gating
    output reg  [31:0] period,        // cycles between accepted ZC events
    output reg         valid
);

    // 2-flop synchronizer for comparator
    reg sig_meta, sig_sync, sig_sync_d;
    always @(posedge clk) begin
        if (rst) begin
            sig_meta   <= 1'b0;
            sig_sync   <= 1'b0;
            sig_sync_d <= 1'b0;
        end else begin
            sig_meta   <= sig_in;
            sig_sync   <= sig_meta;
            sig_sync_d <= sig_sync;
        end
    end

    wire sig_fall = ~sig_sync & sig_sync_d;

    reg [31:0] zc_counter;

    always @(posedge clk) begin
        if (rst) begin
            zc_counter <= 32'd0;
            period     <= 32'd0;
            valid      <= 1'b0;
        end else begin
            valid <= 1'b0;

            // free-run counter (or only run while in relevant sector - your choice)
            zc_counter <= zc_counter + 32'd1;

            // accept ZC only in the intended sector/state
            if (mos_state[1] && sig_fall) begin
                period     <= zc_counter;
                valid      <= 1'b1;
                zc_counter <= 32'd0;
            end
        end
    end
endmodule

