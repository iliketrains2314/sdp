// Verilog-2001 synthesizable version (no SystemVerilog keywords)
// FULL FILE with ALIGN support added for open-loop TOP
//
// New feature:
//   - align_en input (intended to come from your repurposed GPIO bit1)
//   - When align_en=1 and speed>0 in open-loop TOP:
//       * commutation_step is held at ALIGN_STEP
//       * step_counter is held at 0
//
// Notes:
//   - ALIGN_STEP is a parameter in TOP (default 0).
//   - Closed-loop TOP_SENSORLESS already has its own ALIGN state; align_en is not used there.

module motor_controller_top(
    input  wire        sys_clock,              // 100MHz
    input  wire        cpu_reset_n,

    // AXI GPIO0 - Speed and Torque
    input  wire [15:0] gpio0_ch1,               // Speed
    input  wire [15:0] gpio0_ch2,               // Torque (PWM peak)

    // AXI GPIO2 - Control signals
    input  wire [7:0]  gpio2_ch1,               // Trap percentage (0-100)
    input  wire        gpio2_ch2,               // Direction

    // NEW: Align enable (use your former "bemf" control bit for this)
    input  wire        align_en,

    // Control mode selection
    input  wire        enable_bemf,             // 0=open-loop, 1=closed-loop BEMF
    input  wire [2:0]  bemf_comparators,

    // Motor outputs
    output wire [5:0]  mosfet_out,
    output wire        clk_10mhz
);

    wire reset;
    assign reset = ~cpu_reset_n;

    // Clock divider: 100MHz -> ~10MHz (toggle output every 5 cycles => 100/(2*5)=10MHz)
    clock_divider clk_div (
        .clk_100mhz(sys_clock),
        .rst(reset),
        .clk_10mhz(clk_10mhz)
    );

    // Hybrid motor controller
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


// Hybrid H-bridge controller: open-loop PWM trapezoid OR closed-loop sensorless (no PWM)
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
    wire [5:0] closed_loop_out;

    // Open-loop controller (PWM + trapezoid shaping on upper devices)
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

    // Closed-loop sensorless controller (direct on/off)
    TOP_SENSORLESS closed_loop_ctrl (
        .clk(clk),
        .rst(rst),
        .speed(speed),
        .direction(direction),
        .bemf_comparators(bemf_comparators),
        .mosfet_out(closed_loop_out)
    );

    assign mosfet_out = enable_bemf ? closed_loop_out : open_loop_out;

endmodule


// Open-loop controller (PWM trapezoid shaping on upper MOSFETs)
// Verilog-2001 version (reg/wire + always @)
module TOP(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] speed,          // Controls commutation frequency
    input  wire [15:0] torque,         // Controls peak PWM duty cycle
    input  wire [7:0]  trap_percent,   // Trapezoidal ramp percentage (0-100)
    input  wire        direction,      // 0=forward, 1=reverse
    input  wire        align_en,       // NEW: hold commutation at ALIGN_STEP
    output reg  [5:0]  mosfet_out
);

    // Which commutation vector to hold during align
    parameter [2:0] ALIGN_STEP = 3'd0;

    // PWM generation
    reg  [9:0]  pwm_counter;
    wire        pwm_signal;

    // Step counter and timing
    reg  [15:0] step_counter;
    reg  [2:0]  commutation_step;
    reg  [2:0]  prev_commutation_step;

    // Inverted step period - higher speed = faster commutation
    wire [15:0] step_period;
    assign step_period = (16'hFFFF - speed) + 16'h0001;

    // Reset PWM counter at the start of each new commutation step
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

    // Precompute thresholds combinationally (32-bit math)
    always @(*) begin
        ramp_up_threshold   = ( {16'd0, step_period} * {24'd0, clamped_trap_percent} ) / 32'd100;
        ramp_down_threshold = {16'd0, step_period} -
                             ( ( {16'd0, step_period} * {24'd0, clamped_trap_percent} ) / 32'd100 );
    end

    // Duty profile across the commutation step
    always @(*) begin
        if (speed == 16'd0) begin
            pwm_duty = 10'd0;
        end else if (clamped_trap_percent == 8'd0) begin
            pwm_duty = torque[15:6]; // square
        end else if (step_counter < ramp_up_threshold[15:0]) begin
            // Ramp-up
            if (ramp_up_threshold == 32'd0) begin
                pwm_duty = torque[15:6];
            end else begin
                temp_duty = ( {16'd0, step_counter} * {22'd0, torque[15:6]} ) / ramp_up_threshold;
                pwm_duty  = temp_duty[9:0];
            end
        end else if (step_counter < ramp_down_threshold[15:0]) begin
            // Flat-top
            pwm_duty = torque[15:6];
        end else begin
            // Ramp-down
            if ( (step_period - ramp_down_threshold[15:0]) == 16'd0 ) begin
                pwm_duty = 10'd0;
            end else begin
                temp_duty = ( {16'd0, (step_period - step_counter)} * {22'd0, torque[15:6]} ) /
                            ( {16'd0, step_period} - ramp_down_threshold );
                pwm_duty  = temp_duty[9:0];
            end
        end
    end

    assign pwm_signal = (pwm_counter < pwm_duty) ? 1'b1 : 1'b0;

    // Step timing - determines commutation frequency
    // NEW: if align_en=1 and speed>0 -> hold commutation_step at ALIGN_STEP
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

            end else if (step_counter >= step_period) begin
                step_counter <= 16'd0;
                if (direction) begin
                    // reverse
                    commutation_step <= (commutation_step == 3'd0) ? 3'd5 : (commutation_step - 3'd1);
                end else begin
                    // forward
                    commutation_step <= (commutation_step == 3'd5) ? 3'd0 : (commutation_step + 3'd1);
                end
            end else begin
                step_counter <= step_counter + 16'd1;
            end
        end
    end

    // 6-step trapezoidal commutation pattern
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

    // Apply PWM to upper MOSFETs
    always @(*) begin
        if (speed == 16'd0) begin
            mosfet_out = 6'b000_000;
        end else begin
            mosfet_out[5:3] = commutation_pattern[5:3];
            mosfet_out[2:0] = commutation_pattern[2:0]& {3{pwm_signal}};
        end
    end

endmodule


// Closed-loop sensorless controller (direct on/off, no PWM) - already Verilog-2001 style
module TOP_SENSORLESS(
    input  wire        clk,
    input  wire        rst,
    input  wire [15:0] speed,
    input  wire        direction,
    input  wire [2:0]  bemf_comparators,
    output reg  [5:0]  mosfet_out
);

    parameter [2:0] IDLE    = 3'd0;
    parameter [2:0] ALIGN   = 3'd1;
    parameter [2:0] RAMPUP  = 3'd2;
    parameter [2:0] RUNNING = 3'd3;
    parameter [2:0] ERROR   = 3'd4;

    reg [2:0]  state;
    reg [2:0]  commutation_step;
    reg [15:0] state_timer;
    reg [15:0] bemf_timer;
    reg [15:0] commutation_period;
    reg [15:0] last_period;
    reg [7:0]  startup_step_count;
    reg [2:0]  bemf_sync1, bemf_sync2, bemf_sync3;
    reg        bemf_detected;
    reg        expected_bemf_state;

    parameter [15:0] ALIGN_TIME    = 16'd25000;
    parameter [7:0]  STARTUP_STEPS = 8'd30;
    parameter [15:0] MIN_PERIOD    = 16'd500;
    parameter [15:0] MAX_PERIOD    = 16'd50000;
    parameter [15:0] BEMF_DELAY    = 16'd150;

    // Synchronize comparators
    always @(posedge clk) begin
        bemf_sync1 <= bemf_comparators;
        bemf_sync2 <= bemf_sync1;
        bemf_sync3 <= bemf_sync2;
    end

    // Expected BEMF state per step
    always @(*) begin
        case (commutation_step)
            3'd0: expected_bemf_state = direction ? ~bemf_sync3[2] :  bemf_sync3[2];
            3'd1: expected_bemf_state = direction ?  bemf_sync3[1] : ~bemf_sync3[1];
            3'd2: expected_bemf_state = direction ? ~bemf_sync3[0] :  bemf_sync3[0];
            3'd3: expected_bemf_state = direction ?  bemf_sync3[2] : ~bemf_sync3[2];
            3'd4: expected_bemf_state = direction ? ~bemf_sync3[1] :  bemf_sync3[1];
            3'd5: expected_bemf_state = direction ?  bemf_sync3[0] : ~bemf_sync3[0];
            default: expected_bemf_state = 1'b0;
        endcase
    end

    always @(*) begin
        bemf_detected = expected_bemf_state && (bemf_timer > BEMF_DELAY);
    end

    // State machine
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state              <= IDLE;
            commutation_step   <= 3'd0;
            state_timer        <= 16'd0;
            bemf_timer         <= 16'd0;
            commutation_period <= 16'd10000;
            startup_step_count <= 8'd0;
            last_period        <= 16'd0;
        end else begin
            case (state)
                IDLE: begin
                    commutation_step   <= 3'd0;
                    state_timer        <= 16'd0;
                    bemf_timer         <= 16'd0;
                    startup_step_count <= 8'd0;
                    if (speed > 16'd0) begin
                        state <= ALIGN;
                        commutation_period <= (16'hFFFF - speed) + 16'h0001;
                    end
                end

                ALIGN: begin
                    commutation_step <= 3'd0;
                    if (state_timer >= ALIGN_TIME) begin
                        state <= RAMPUP;
                        state_timer <= 16'd0;
                        startup_step_count <= 8'd0;
                    end else begin
                        state_timer <= state_timer + 16'd1;
                    end
                end

                RAMPUP: begin
                    state_timer <= state_timer + 16'd1;
                    if (state_timer >= commutation_period) begin
                        state_timer <= 16'd0;
                        bemf_timer  <= 16'd0;
                        startup_step_count <= startup_step_count + 8'd1;

                        if (direction) begin
                            commutation_step <= (commutation_step == 3'd0) ? 3'd5 : (commutation_step - 3'd1);
                        end else begin
                            commutation_step <= (commutation_step == 3'd5) ? 3'd0 : (commutation_step + 3'd1);
                        end

                        if (commutation_period > MIN_PERIOD) begin
                            commutation_period <= commutation_period - (commutation_period >> 5);
                        end

                        if (startup_step_count >= STARTUP_STEPS) begin
                            state <= RUNNING;
                            last_period <= commutation_period;
                        end
                    end
                end

                RUNNING: begin
                    bemf_timer <= bemf_timer + 16'd1;

                    if (bemf_detected && (bemf_timer >= (last_period >> 1))) begin
                        if (direction) begin
                            commutation_step <= (commutation_step == 3'd0) ? 3'd5 : (commutation_step - 3'd1);
                        end else begin
                            commutation_step <= (commutation_step == 3'd5) ? 3'd0 : (commutation_step + 3'd1);
                        end
                        last_period <= bemf_timer;
                        bemf_timer  <= 16'd0;
                    end

                    if (bemf_timer > MAX_PERIOD) begin
                        state <= ERROR;
                    end

                    if (speed == 16'd0) begin
                        state <= IDLE;
                    end
                end

                ERROR: begin
                    commutation_step <= 3'd0;
                    if (speed == 16'd0) begin
                        state <= IDLE;
                    end
                end

                default: state <= IDLE;
            endcase
        end
    end

    // Commutation decode
    reg [5:0] commutation_pattern;

    always @(*) begin
        case (commutation_step)
            3'd0: commutation_pattern = 6'b100_010;
            3'd1: commutation_pattern = 6'b100_001;
            3'd2: commutation_pattern = 6'b010_001;
            3'd3: commutation_pattern = 6'b010_100;
            3'd4: commutation_pattern = 6'b001_100;
            3'd5: commutation_pattern = 6'b001_010;
            default: commutation_pattern = 6'b000_000;
        endcase
    end

    always @(*) begin
        if (state == IDLE || state == ERROR) begin
            mosfet_out = 6'b000_000;
        end else begin
            mosfet_out = commutation_pattern;
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
