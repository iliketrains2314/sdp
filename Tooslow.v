// Verilog-2001 synthesizable version
// Updated for:
// - align_en support
// - low-side PWM (better startup/bootstrap behavior)
// - slower commutation timing using a separate prescaler
// - open-loop only (closed-loop block still commented out)

module motor_controller_top(
    input  wire        sys_clock,              // 100MHz
    input  wire        cpu_reset_n,

    // AXI GPIO0 - Speed and Torque
    input  wire [15:0] gpio0_ch1,              // Speed
    input  wire [15:0] gpio0_ch2,              // Torque

    // AXI GPIO2 - Control signals
    input  wire [7:0]  gpio2_ch1,              // Trap percentage (0-100)
    input  wire        gpio2_ch2,              // Direction

    // Align control
    input  wire        align_en,

    // Unused for now, kept for compatibility
    input  wire        enable_bemf,
    input  wire [2:0]  bemf_comparators,

    // Motor outputs
    output wire [5:0]  mosfet_out,
    output wire        clk_10mhz
);

    wire reset;
    assign reset = ~cpu_reset_n;

    clock_divider clk_div (
        .clk_100mhz(sys_clock),
        .rst(reset),
        .clk_10mhz(clk_10mhz)
    );

    hbridge motor_driver (
        .clk(clk_10mhz),
        .rst(reset),
        .speed(gpio0_ch1),
        .torque(gpio0_ch2),
        .trap_percent(gpio2_ch1),
        .direction(gpio2_ch2),
        .align_en(align_en),
        .enable_bemf(enable_bemf),
        .bemf_comparators(bemf_comparators),
        .mosfet_out(mosfet_out)
    );

endmodule


module hbridge(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] speed,
    input  wire [15:0] torque,
    input  wire [7:0]  trap_percent,
    input  wire        direction,
    input  wire        align_en,
    input  wire        enable_bemf,
    input  wire [2:0]  bemf_comparators,
    output wire [5:0]  mosfet_out
);

    wire [5:0] open_loop_out;

    TOP open_loop_ctrl (
        .clk(clk),
        .rst(rst),
        .speed(speed),
        .torque(torque),
        .trap_percent(trap_percent),
        .direction(direction),
        .align_en(align_en),
        .mosfet_out(open_loop_out)
    );

    // Closed-loop block intentionally not used right now
    assign mosfet_out = open_loop_out;

endmodule


module TOP(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] speed,
    input  wire [15:0] torque,
    input  wire [7:0]  trap_percent,
    input  wire        direction,
    input  wire        align_en,
    output reg  [5:0]  mosfet_out
);

    parameter [2:0] ALIGN_STEP = 3'd0;

    // PWM generation
    reg  [9:0] pwm_counter;
    wire       pwm_signal;

    // Commutation timing
    reg  [15:0] step_counter;
    reg  [2:0]  commutation_step;
    reg  [2:0]  prev_commutation_step;

    // Separate commutation prescaler
    reg  [7:0] comm_prescaler;
    wire       comm_tick;

    // Inverted step period - higher speed = faster commutation
    wire [15:0] step_period;
    assign step_period = (16'hFFFF - speed) + 16'h0001;

    // Make commutation much slower than PWM
    // comm_tick occurs once every 256 clk cycles
    assign comm_tick = (comm_prescaler == 8'd255);

    // Prescaler runs continuously
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            comm_prescaler <= 8'd0;
        end else begin
            comm_prescaler <= comm_prescaler + 8'd1;
        end
    end

    // PWM counter resets when commutation step changes
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            pwm_counter           <= 10'd0;
            prev_commutation_step <= 3'd0;
        end else begin
            if (commutation_step != prev_commutation_step) begin
                pwm_counter           <= 10'd0;
                prev_commutation_step <= commutation_step;
            end else begin
                pwm_counter <= pwm_counter + 10'd1;
            end
        end
    end

    // Trapezoidal shaping duty calculation
    reg  [9:0]  pwm_duty;
    wire [7:0]  clamped_trap_percent;
    reg  [31:0] ramp_up_threshold;
    reg  [31:0] ramp_down_threshold;
    reg  [31:0] temp_duty;

    assign clamped_trap_percent = (trap_percent > 8'd50) ? 8'd50 : trap_percent;

    always @(*) begin
        ramp_up_threshold   = ({16'd0, step_period} * {24'd0, clamped_trap_percent}) / 32'd100;
        ramp_down_threshold = {16'd0, step_period} -
                              (({16'd0, step_period} * {24'd0, clamped_trap_percent}) / 32'd100);
    end

    always @(*) begin
        if (speed == 16'd0) begin
            pwm_duty = 10'd0;
        end else if (clamped_trap_percent == 8'd0) begin
            pwm_duty = torque[15:6];
        end else if (step_counter < ramp_up_threshold[15:0]) begin
            if (ramp_up_threshold == 32'd0) begin
                pwm_duty = torque[15:6];
            end else begin
                temp_duty = ({16'd0, step_counter} * {22'd0, torque[15:6]}) / ramp_up_threshold;
                pwm_duty  = temp_duty[9:0];
            end
        end else if (step_counter < ramp_down_threshold[15:0]) begin
            pwm_duty = torque[15:6];
        end else begin
            if ((step_period - ramp_down_threshold[15:0]) == 16'd0) begin
                pwm_duty = 10'd0;
            end else begin
                temp_duty = ({16'd0, (step_period - step_counter)} * {22'd0, torque[15:6]}) /
                            ({16'd0, step_period} - ramp_down_threshold);
                pwm_duty  = temp_duty[9:0];
            end
        end
    end

    assign pwm_signal = (pwm_counter < pwm_duty) ? 1'b1 : 1'b0;

    // Slower commutation timing
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            step_counter     <= 16'd0;
            commutation_step <= 3'd0;
        end else begin
            if (speed == 16'd0) begin
                step_counter     <= 16'd0;
                commutation_step <= 3'd0;
            end else if (align_en) begin
                step_counter     <= 16'd0;
                commutation_step <= ALIGN_STEP;
            end else if (comm_tick) begin
                if (step_counter >= step_period) begin
                    step_counter <= 16'd0;
                    if (direction) begin
                        commutation_step <= (commutation_step == 3'd0) ? 3'd5 : (commutation_step - 3'd1);
                    end else begin
                        commutation_step <= (commutation_step == 3'd5) ? 3'd0 : (commutation_step + 3'd1);
                    end
                end else begin
                    step_counter <= step_counter + 16'd1;
                end
            end
        end
    end

    // 6-step commutation pattern
    reg [5:0] commutation_pattern;

    always @(*) begin
        case (commutation_step)
            3'd0: commutation_pattern = 6'b100_010; // UH-VL
            3'd1: commutation_pattern = 6'b100_001; // UH-WL
            3'd2: commutation_pattern = 6'b010_001; // VH-WL
            3'd3: commutation_pattern = 6'b010_100; // VH-UL
            3'd4: commutation_pattern = 6'b001_100; // WH-UL
            3'd5: commutation_pattern = 6'b001_010; // WH-VL
            default: commutation_pattern = 6'b000_000;
        endcase
    end

    // Apply PWM to LOWER MOSFETs
    always @(*) begin
        if (speed == 16'd0) begin
            mosfet_out = 6'b000_000;
        end else begin
            mosfet_out[5:3] = commutation_pattern[5:3];                 // high-side steady
            mosfet_out[2:0] = commutation_pattern[2:0] & {3{pwm_signal}}; // low-side PWM
        end
    end

endmodule


module clock_divider (
    input  wire clk_100mhz,
    input  wire rst,
    output reg  clk_10mhz
);
    parameter DIVISOR = 5;
    reg [2:0] counter;

    always @(posedge clk_100mhz or posedge rst) begin
        if (rst) begin
            counter   <= 3'd0;
            clk_10mhz <= 1'b0;
        end else begin
            if (counter == (DIVISOR - 1)) begin
                counter   <= 3'd0;
                clk_10mhz <= ~clk_10mhz;
            end else begin
                counter <= counter + 3'd1;
            end
        end
    end
endmodule
