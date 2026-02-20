module motor_controller_top(
    input wire sys_clock,              // 100MHz
    input wire cpu_reset_n,
    
    // AXI GPIO0 - Speed and Torque (torque unused internally)
    input wire [15:0] gpio0_ch1,       // Speed
    input wire [15:0] gpio0_ch2,       // Torque (unused)
    
    // AXI GPIO2 - Control signals
    input wire [7:0] gpio2_ch1,        // Trap percentage (unused)
    input wire gpio2_ch2,              // Direction
    
    // Control mode selection
    input wire enable_bemf,            // 0=open-loop, 1=closed-loop BEMF
    input wire [2:0] bemf_comparators, 
 
    // Motor outputs
    output wire [5:0] mosfet_out,
    output wire clk_10mhz
);

    wire reset;
    
    assign reset = !cpu_reset_n;
    
    // Clock divider
    clock_divider clk_div (
        .clk_100mhz(sys_clock),
        .clk_10mhz(clk_10mhz)
    );
    
    // Hybrid motor controller
    hbridge motor_driver (
        .clk(clk_10mhz),
        .rst(reset),
        .speed(gpio0_ch1),
        .direction(gpio2_ch2),
        .enable_bemf(enable_bemf),
        .bemf_comparators(bemf_comparators),
        .mosfet_out(mosfet_out)
    );

endmodule


// Hybrid H-bridge controller (no PWM, just on/off)
module hbridge(
    input wire clk,
    input wire rst,
    input wire [15:0] speed,
    input wire direction,
    input wire enable_bemf,
    input wire [2:0] bemf_comparators,
    output wire [5:0] mosfet_out
);

    // Outputs from both controllers
    wire [5:0] open_loop_out;
    wire [5:0] closed_loop_out;
    
    // Open-loop controller
    TOP open_loop_ctrl (
        .clk(clk),
        .rst(rst),
        .speed(speed),
        .direction(direction),
        .mosfet_out(open_loop_out)
    );
    
    // Closed-loop sensorless controller
    TOP_SENSORLESS closed_loop_ctrl (
        .clk(clk),
        .rst(rst),
        .speed(speed),
        .direction(direction),
        .bemf_comparators(bemf_comparators),
        .mosfet_out(closed_loop_out)
    );
    
    // Mode multiplexer
    assign mosfet_out = enable_bemf ? closed_loop_out : open_loop_out;

endmodule

// Original open-loop controller (NO PWM - just on/off)
module TOP(
    input wire clk,
    input wire rst,
    input wire [15:0] speed,
    input wire direction,
    output reg [5:0] mosfet_out
);

    reg [15:0] step_counter;
    reg [2:0] commutation_step;
    wire [15:0] step_period;
    
    assign step_period = (16'hFFFF - speed) + 1;
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            step_counter <= 16'd0;
            commutation_step <= 3'd0;
        end else begin
            if (speed == 16'd0) begin
                step_counter <= 16'd0;
                commutation_step <= 3'd0;
            end else if (step_counter >= step_period) begin
                step_counter <= 16'd0;
                if (direction == 1'b0) begin
                    commutation_step <= (commutation_step == 3'd5) ? 3'd0 : commutation_step + 1;
                end else begin
                    commutation_step <= (commutation_step == 3'd0) ? 3'd5 : commutation_step - 1;
                end
            end else begin
                step_counter <= step_counter + 1;
            end
        end
    end
    
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
        if (speed == 16'd0) begin
            mosfet_out = 6'b000_000;
        end else begin
            mosfet_out = commutation_pattern;  // Direct output, no PWM
        end
    end

endmodule

// Closed-loop sensorless controller (NO PWM - just on/off)
module TOP_SENSORLESS(
    input wire clk,
    input wire rst,
    input wire [15:0] speed,
    input wire direction,
    input wire [2:0] bemf_comparators,
    output reg [5:0] mosfet_out
);

    parameter [2:0] IDLE = 3'd0;
    parameter [2:0] ALIGN = 3'd1;
    parameter [2:0] RAMPUP = 3'd2;
    parameter [2:0] RUNNING = 3'd3;
    parameter [2:0] ERROR = 3'd4;
    
    reg [2:0] state;
    reg [2:0] commutation_step;
    reg [15:0] state_timer;
    reg [15:0] bemf_timer;
    reg [15:0] commutation_period;
    reg [15:0] last_period;
    reg [7:0] startup_step_count;
    reg [2:0] bemf_sync1, bemf_sync2, bemf_sync3;
    reg bemf_detected, expected_bemf_state;
    
    parameter ALIGN_TIME = 16'd25000;
    parameter STARTUP_STEPS = 8'd30;
    parameter MIN_PERIOD = 16'd500;
    parameter MAX_PERIOD = 16'd50000;
    parameter BEMF_DELAY = 16'd150;
    
    always @(posedge clk) begin
        bemf_sync1 <= bemf_comparators;
        bemf_sync2 <= bemf_sync1;
        bemf_sync3 <= bemf_sync2;
    end
    
    always @(*) begin
        case (commutation_step)
            3'd0: expected_bemf_state = direction ? ~bemf_sync3[2] : bemf_sync3[2];
            3'd1: expected_bemf_state = direction ? bemf_sync3[1] : ~bemf_sync3[1];
            3'd2: expected_bemf_state = direction ? ~bemf_sync3[0] : bemf_sync3[0];
            3'd3: expected_bemf_state = direction ? bemf_sync3[2] : ~bemf_sync3[2];
            3'd4: expected_bemf_state = direction ? ~bemf_sync3[1] : bemf_sync3[1];
            3'd5: expected_bemf_state = direction ? bemf_sync3[0] : ~bemf_sync3[0];
            default: expected_bemf_state = 1'b0;
        endcase
    end
    
    always @(*) begin
        bemf_detected = expected_bemf_state && (bemf_timer > BEMF_DELAY);
    end
    
    always @(posedge clk or posedge rst) begin
        if (rst) begin
            state <= IDLE;
            commutation_step <= 3'd0;
            state_timer <= 16'd0;
            bemf_timer <= 16'd0;
            commutation_period <= 16'd10000;
            startup_step_count <= 8'd0;
            last_period <= 16'd0;
        end else begin
            case (state)
                IDLE: begin
                    commutation_step <= 3'd0;
                    state_timer <= 16'd0;
                    bemf_timer <= 16'd0;
                    startup_step_count <= 8'd0;
                    if (speed > 16'd0) begin
                        state <= ALIGN;
                        commutation_period <= (16'hFFFF - speed) + 1;
                    end
                end
                
                ALIGN: begin
                    commutation_step <= 3'd0;
                    if (state_timer >= ALIGN_TIME) begin
                        state <= RAMPUP;
                        state_timer <= 16'd0;
                        startup_step_count <= 8'd0;
                    end else begin
                        state_timer <= state_timer + 1;
                    end
                end
                
                RAMPUP: begin
                    state_timer <= state_timer + 1;
                    if (state_timer >= commutation_period) begin
                        state_timer <= 16'd0;
                        bemf_timer <= 16'd0;
                        startup_step_count <= startup_step_count + 1;
                        
                        if (direction) begin
                            commutation_step <= (commutation_step == 3'd0) ? 3'd5 : commutation_step - 1;
                        end else begin
                            commutation_step <= (commutation_step == 3'd5) ? 3'd0 : commutation_step + 1;
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
                    bemf_timer <= bemf_timer + 1;
                    if (bemf_detected && (bemf_timer >= (last_period >> 1))) begin
                        if (direction) begin
                            commutation_step <= (commutation_step == 3'd0) ? 3'd5 : commutation_step - 1;
                        end else begin
                            commutation_step <= (commutation_step == 3'd5) ? 3'd0 : commutation_step + 1;
                        end
                        last_period <= bemf_timer;
                        bemf_timer <= 16'd0;
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
            mosfet_out = commutation_pattern;  // Direct output, no PWM
        end
    end

endmodule

module clock_divider (
    input wire clk_100mhz,
    output reg clk_10mhz
);
    parameter DIVISOR = 5;
    reg [2:0] counter;
    
    always @(posedge clk_100mhz) begin
        if (counter == DIVISOR - 1) begin
            counter <= 0;
            clk_10mhz <= ~clk_10mhz;
        end else begin
            counter <= counter + 1;
        end
    end
endmodule