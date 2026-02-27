`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// 16-bit PID with runtime gains (fractional) and sample frequency
//////////////////////////////////////////////////////////////////////////////////

module PID #(
    parameter integer CLK_FREQ     = 100_000_000,   // FPGA clock in Hz
    parameter integer GAIN_FRAC    = 12,            // frac bits for kp/ki/kd
    parameter integer max_setpoint = 200,
    parameter integer OUT_SHIFT    = 0              // (unused in this version)
)
(
    input  wire        clk,              // input clk
    input  wire        rst,              // reset (sync)
    input  wire [15:0] setpoint,         // setpoint for speed
    input  wire [15:0] feedback,         // feedback input
    input  wire        clear_i,          // clears built-up integration summation
    input  wire        sel,              // open-loop/closed-loop select

    // Fractional PID gains as fixed-point inputs (Q(GAIN_FRAC))
    input  wire signed [15:0] kp_in,
    input  wire signed [15:0] ki_in,
    input  wire signed [15:0] kd_in,

    // Runtime-configurable sample frequency in Hz
    input  wire [31:0] sample_freq,

    // 16-bit corrected speed setpoint output (PID result)
    output reg  [15:0] speedpoint,

    // Wide debug/monitor output = setpoint + total_error_fx
    output wire signed [47:0] total
);

// ---------------------------------------------------------------------------
// Constants
// ---------------------------------------------------------------------------
localparam signed [31:0] I_MIN = -32'sd100_000_000;
localparam signed [31:0] I_MAX =  32'sd100_000_000;

// ---------------------------------------------------------------------------
// Sample timing
// ---------------------------------------------------------------------------
reg [31:0] sample_count;
reg [31:0] sample_per;
reg        sample_pulse;

// ---------------------------------------------------------------------------
// PID state
// ---------------------------------------------------------------------------
reg  signed [16:0] error, prev_error;
reg  signed [31:0] integral;
reg  signed [31:0] i_next;
reg  signed [16:0] derivative;

reg  signed [47:0] p_term, i_term, d_term;
reg  signed [47:0] total_error_fx;   // fixed-point result after gain scaling

// Widened fixed-point gains for internal math (32-bit signed)
reg signed [31:0] kp, ki, kd;

// ---------------------------------------------------------------------------
// Sign-extend 16-bit fixed-point gains to 32 bits (combinational)
// ---------------------------------------------------------------------------
always @* begin
    kp = { {16{kp_in[15]}}, kp_in };
    ki = { {16{ki_in[15]}}, ki_in };
    kd = { {16{kd_in[15]}}, kd_in };
end

// ---------------------------------------------------------------------------
// multiplication helper: apply fixed-point gain and re-align
// val is integer-domain, gain_s is Q(GAIN_FRAC); result back in val-scale
// ---------------------------------------------------------------------------
function signed [47:0] mul_gain_align;
    input signed [31:0] val;     // value in integer domain
    input signed [31:0] gain_s;  // fixed-point gain
    reg   signed [63:0] prod;
begin
    prod = val * gain_s;
    // Align back by GAIN_FRAC to return to val's scale
    mul_gain_align = prod >>> GAIN_FRAC;
end
endfunction

// ---------------------------------------------------------------------------
// Sample period computation
// sample_per = CLK_FREQ / sample_freq
// ---------------------------------------------------------------------------
always @* begin
    if (sample_freq != 0)
        sample_per = CLK_FREQ / sample_freq;
    else
        sample_per = 32'd1;  // avoid div-by-zero; fastest update
end

// Sample timer
always @(posedge clk) begin
    if (rst) begin
        sample_count <= 32'd0;
        sample_pulse <= 1'b0;
    end else begin
        if (sample_count >= (sample_per - 1)) begin
            sample_count <= 32'd0;
            sample_pulse <= 1'b1;
        end else begin
            sample_count <= sample_count + 32'd1;
            sample_pulse <= 1'b0;
        end
    end
end

// ---------------------------------------------------------------------------
// PID core: error, integral (with anti-windup), derivative, P/I/D terms
// ---------------------------------------------------------------------------
always @(posedge clk) begin
    if (rst) begin
        prev_error     <= 17'sd0;
        error          <= 17'sd0;
        integral       <= 32'sd0;
        derivative     <= 17'sd0;
        total_error_fx <= 48'sd0;
        i_next         <= 32'sd0;
        p_term         <= 48'sd0;
        i_term         <= 48'sd0;
        d_term         <= 48'sd0;
    end else if (sample_pulse) begin
        // 1) Error and derivative
        error      <= $signed({1'b0, setpoint}) - $signed({1'b0, feedback});
        derivative <= error - prev_error;

        // 2) Integrator with clear and anti-windup
        if (clear_i) begin
            integral <= 32'sd0;
        end else begin
            // sign-extend error (17 -> 32 bits)
            i_next <= integral + { {15{error[16]}}, error };
            if      (i_next > I_MAX) integral <= I_MAX;
            else if (i_next < I_MIN) integral <= I_MIN;
            else                     integral <= i_next;
        end

        // 3) P, I, D terms with fixed-point gains from inputs
        p_term <= mul_gain_align({ {15{error[16]}},      error},     kp);
        i_term <= mul_gain_align( integral,                           ki);
        d_term <= mul_gain_align({ {15{derivative[16]}}, derivative}, kd);

        // 4) Sum of PID terms
        total_error_fx <= p_term + i_term + d_term;

        // 5) Save previous error for derivative
        prev_error <= error;
    end
end

// ---------------------------------------------------------------------------
// Generate 48-bit corrected setpoint (for debug) and 16-bit speedpoint output
// ---------------------------------------------------------------------------

wire signed [47:0] corrected_wide;
reg  signed [47:0] tmp;

// Wide version: original setpoint + PID correction
assign corrected_wide = $signed(total_error_fx) + $signed({32'd0, setpoint});

// Export full-precision value as 'total' (debug/monitor)
assign total = corrected_wide;

// 16-bit saturated speedpoint output
always @* begin
    tmp = corrected_wide;

    if (sel == 1'b0) begin
        // open loop: just pass the commanded setpoint
        speedpoint = setpoint;
    end else begin
        // closed loop: saturated PID result
        if (tmp > 48'sd65535)
            speedpoint = 16'd65535;
        else if (tmp < 0)
            speedpoint = 16'd0;
        else
            speedpoint = tmp[15:0];  // lower 16 bits, positive
    end
end

endmodule
