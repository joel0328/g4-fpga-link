// hbridge_pwm.v
// One sign-magnitude H-bridge channel for a TB6612FNG-style driver with its
// PWM pin tied high: the PWM goes on IN1 (forward) or IN2 (reverse) and the
// other input stays low, so the off time coasts. IN1 = IN2 = 0 is coast.
//
// duty is signed, +/-32767 = +/-100 % (-32768 is treated as -32767); its
// magnitude is clamped to limit. A new duty or direction takes effect at the
// next period boundary, so a pulse is never cut short or stretched, and every
// period ends with at least one clock off (the most is PERIOD-1 clocks on).
// enable (ARMED) gates both pins directly: dropping it turns them off on the
// same clock, mid-period.

module hbridge_pwm #(
    parameter integer PERIOD = 5000          // clocks per PWM period (< 65536)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        enable,
    input  wire [15:0] duty,                 // signed
    input  wire [14:0] limit,
    output wire        in1,
    output wire        in2
);

    reg [15:0] cnt     = 16'd0;
    reg [14:0] mag_q   = 15'd0;              // pipeline: clamp, then multiply
    reg        neg_q   = 1'b0;
    reg [15:0] thr     = 16'd0;              // clocks on, from the current duty
    reg        neg     = 1'b0;
    reg [15:0] thr_act = 16'd0;              // latched at the period boundary
    reg        neg_act = 1'b0;
    reg        pwm_q   = 1'b0;

    wire [15:0] mag   = duty[15] ? (16'd0 - duty) : duty;   // 0..32768
    wire [14:0] mag_c = (mag > {1'b0, limit}) ? limit : mag[14:0];
    wire [31:0] prod  = mag_q * PERIOD;

    always @(posedge clk) begin
        if (rst) begin
            cnt     <= 16'd0;
            mag_q   <= 15'd0;
            neg_q   <= 1'b0;
            thr     <= 16'd0;
            neg     <= 1'b0;
            thr_act <= 16'd0;
            neg_act <= 1'b0;
            pwm_q   <= 1'b0;
        end else begin
            mag_q <= mag_c;
            neg_q <= duty[15];
            thr   <= prod[30:15];                // mag_q * PERIOD / 32768
            neg   <= neg_q;
            if (cnt == PERIOD - 1) begin
                cnt     <= 16'd0;
                thr_act <= thr;
                neg_act <= neg;
            end else begin
                cnt <= cnt + 16'd1;
            end
            pwm_q <= (cnt < thr_act);
        end
    end

    assign in1 = enable && pwm_q && !neg_act;
    assign in2 = enable && pwm_q &&  neg_act;

endmodule
