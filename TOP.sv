module TOP(
    input logic clk,
    input logic rst,
    input logic [15:0] speed,          // Controls commutation frequency
    input logic [15:0] torque,         // Controls peak PWM duty cycle
    input logic [7:0] trap_percent,    // Trapezoidal ramp percentage (0-100)
    output logic [5:0] mosfet_out
);

    // PWM generation
    logic [9:0] pwm_counter;
    logic pwm_signal;
    
    // Step counter and timing
    logic [15:0] step_counter;
    logic [2:0] commutation_step;
    logic [2:0] prev_commutation_step;
    
    // Inverted step period - higher speed = faster commutation
    logic [15:0] step_period;
    assign step_period = (16'hFFFF - speed) + 1;
    
    // Reset PWM counter at the start of each new commutation step
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            pwm_counter <= 10'd0;
            prev_commutation_step <= 3'd0;
        end else begin
            // Detect step change and reset PWM counter for synchronization
            if (commutation_step != prev_commutation_step) begin
                pwm_counter <= 10'd0;
                prev_commutation_step <= commutation_step;
            end else begin
                pwm_counter <= pwm_counter + 1;
            end
        end
    end
    
    // Trapezoidal shaping: Calculate duty cycle based on position within step
    // trap_percent determines the ramp size (0-50%)
    // If trap_percent = 10, then 10% ramp-up, 80% flat, 10% ramp-down
    // If trap_percent = 0, then 0% ramp (pure square wave)
    // If trap_percent = 50, then 50% ramp-up, 0% flat, 50% ramp-down (triangle wave)
    logic [9:0] pwm_duty;
    logic [31:0] ramp_up_threshold;
    logic [31:0] ramp_down_threshold;
    logic [31:0] temp_duty;
    logic [7:0] clamped_trap_percent;
    
    // Clamp trap_percent to maximum of 50%
    assign clamped_trap_percent = (trap_percent > 8'd50) ? 8'd50 : trap_percent;
    
    // Calculate thresholds based on trap_percent
    // ramp_up_threshold = (step_period * trap_percent) / 100
    // ramp_down_threshold = step_period - (step_period * trap_percent) / 100
    assign ramp_up_threshold = ({16'd0, step_period} * {24'd0, clamped_trap_percent}) / 32'd100;
    assign ramp_down_threshold = {16'd0, step_period} - 
                                 (({16'd0, step_period} * {24'd0, clamped_trap_percent}) / 32'd100);
    
    always_comb begin
        if (speed == 16'd0) begin
            pwm_duty = 10'd0;
        end else if (clamped_trap_percent == 8'd0) begin
            // Pure square wave - no ramping
            pwm_duty = torque[15:6];
        end else if (step_counter < ramp_up_threshold[15:0]) begin
            // Ramp-up phase: 0% to 100% of torque setting
            // Use 32-bit math to prevent overflow
            if (ramp_up_threshold == 0) begin
                pwm_duty = torque[15:6];
            end else begin
                temp_duty = ({16'd0, step_counter} * {22'd0, torque[15:6]}) / ramp_up_threshold;
                pwm_duty = temp_duty[9:0];
            end
        end else if (step_counter < ramp_down_threshold[15:0]) begin
            // Flat-top phase: constant at torque setting
            pwm_duty = torque[15:6];
        end else begin
            // Ramp-down phase: 100% to 0% of torque setting
            if ((step_period - ramp_down_threshold[15:0]) == 0) begin
                pwm_duty = 10'd0;
            end else begin
                temp_duty = ({16'd0, (step_period - step_counter)} * {22'd0, torque[15:6]}) / 
                           ({16'd0, step_period} - ramp_down_threshold);
                pwm_duty = temp_duty[9:0];
            end
        end
    end
    
    assign pwm_signal = (pwm_counter < pwm_duty) ? 1'b1 : 1'b0;
    
    // Step timing - determines commutation frequency
    always_ff @(posedge clk or posedge rst) begin
        if (rst) begin
            step_counter <= 16'd0;
            commutation_step <= 3'd0;
        end else begin
            if (speed == 16'd0) begin
                step_counter <= 16'd0;
                commutation_step <= 3'd0;
            end else if (step_counter >= step_period) begin
                step_counter <= 16'd0;
                commutation_step <= (commutation_step == 3'd5) ? 3'd0 : commutation_step + 1;
            end else begin
                step_counter <= step_counter + 1;
            end
        end
    end
    
    // 6-step trapezoidal commutation pattern
    // mosfet_out[5:3] = upper MOSFETs (UH, VH, WH)
    // mosfet_out[2:0] = lower MOSFETs (UL, VL, WL)
    logic [5:0] commutation_pattern;
    
    always_comb begin
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
    
    // Apply PWM to upper MOSFETs to create trapezoidal waveform
    always_comb begin
        if (speed == 16'd0) begin
            mosfet_out = 6'b000_000;
        end else begin
            mosfet_out[5:3] = commutation_pattern[5:3] & {3{pwm_signal}};
            mosfet_out[2:0] = commutation_pattern[2:0];
        end
    end

endmodule
