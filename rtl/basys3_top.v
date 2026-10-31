// basys3_top.v
// Top level for the Nucleo-G431KB <-> Basys 3 link bring-up.
//
// Pmod JA (MCU link):
//   JA1 SPI_SCK  in    JA7  FPGA_IRQ   out (active high)
//   JA2 SPI_MOSI in    JA8  LOOP_TICK  out (1 us high pulse)
//   JA3 SPI_MISO out   JA9  FPGA_RST_N in  (pulled up; MCU drives low to reset)
//   JA4 SPI_CS_N in    JA10 FAULT_N    out (low ~0.1 s after a CRC error, and
//                                         while any safety fault is latched)
//
// The 7-segment display shows the good-frame count until the MCU writes SEG7
// with bit 20 set. LED15 = armed, LED14 = fault (safety layer); LED13..0 come
// from LEDS and are lit only while armed. SW15 is the hardware arm enable and
// btnD the e-stop.
//
// Pmod JB (motors, map v5; TB6612FNG with PWMA/PWMB and STBY tied high):
//   JB1 M1_IN1 out   JB7  ENC1_A in    The IN pins carry 20 kHz PWM, gated by
//   JB2 M1_IN2 out   JB8  ENC1_B in    ARMED: both low (coast) when disarmed.
//   JB3 M2_IN1 out   JB9  ENC2_A in    Encoders run from JB6/JB12 3.3 V,
//   JB4 M2_IN2 out   JB10 ENC2_B in    never 5 V (these pins aren't 5 V tolerant).

module basys3_top (
    input  wire        clk,        // 100 MHz

    input  wire [15:0] sw,
    input  wire        btnC,
    input  wire        btnU,
    input  wire        btnL,
    input  wire        btnR,
    input  wire        btnD,

    output wire [15:0] led,
    output wire [6:0]  seg,
    output wire        dp,
    output wire [3:0]  an,

    input  wire        spi_sck,
    input  wire        spi_mosi,
    output wire        spi_miso,
    input  wire        spi_cs_n,
    output wire        fpga_irq,
    output wire        loop_tick,
    input  wire        fpga_rst_n,
    output wire        fault_n,

    output wire        m1_in1,
    output wire        m1_in2,
    output wire        m2_in1,
    output wire        m2_in2,
    input  wire        enc1_a,
    input  wire        enc1_b,
    input  wire        enc2_a,
    input  wire        enc2_b
);

    `include "fpga_regs.vh"

    // ------------------------------------------------------------------
    // Reset: power-on reset plus MCU-driven reset (synchronized)
    // ------------------------------------------------------------------
    reg [7:0] por_cnt = 8'd0;
    reg [1:0] mcu_rst_s = 2'b11;
    reg       rst = 1'b1;

    always @(posedge clk) begin
        mcu_rst_s <= {mcu_rst_s[0], fpga_rst_n};
        if (por_cnt != 8'hFF) por_cnt <= por_cnt + 8'd1;
        rst <= (por_cnt != 8'hFF) | ~mcu_rst_s[1];
    end

    // ------------------------------------------------------------------
    // Switch and button synchronize + debounce (sampled at 1 kHz)
    // ------------------------------------------------------------------
    reg [16:0] db_div = 17'd0;
    reg [20:0] in_s1 = 21'h0, in_s2 = 21'h0, in_stable = 21'h0;

    always @(posedge clk) begin
        in_s1 <= {btnC, btnD, btnR, btnL, btnU, sw};
        in_s2 <= in_s1;
        if (db_div == 17'd99_999) begin
            db_div    <= 17'd0;
            in_stable <= in_s2;
        end else begin
            db_div <= db_div + 17'd1;
        end
    end

    // ------------------------------------------------------------------
    // SPI link + register file
    // ------------------------------------------------------------------
    wire [6:0]  reg_addr;
    wire [31:0] reg_wdata, reg_rdata;
    wire        reg_we, frame_ok, crc_err, cs_active;
    wire [15:0] seg7_value;
    wire [3:0]  seg7_enable;
    wire        tick_strobe, irq_int, fault_any, armed;
    wire [15:0] m1_duty, m2_duty;
    wire [14:0] duty_limit;
    wire [31:0] enc1_count, enc2_count;
    wire        enc1_err, enc2_err;

    spi_link u_spi (
        .clk(clk), .rst(rst),
        .spi_sck(spi_sck), .spi_cs_n(spi_cs_n),
        .spi_mosi(spi_mosi), .spi_miso(spi_miso),
        .reg_addr(reg_addr), .reg_wdata(reg_wdata),
        .reg_we(reg_we), .reg_rdata(reg_rdata),
        .frame_ok(frame_ok), .crc_err(crc_err), .cs_active(cs_active)
    );

    regfile u_regs (
        .clk(clk), .rst(rst),
        .addr(reg_addr), .wdata(reg_wdata), .we(reg_we), .rdata(reg_rdata),
        .frame_ok(frame_ok), .crc_err(crc_err), .cs_active(cs_active),
        .switches(in_stable[15:0]), .buttons(in_stable[20:16]),
        .leds(led),
        .seg7_value(seg7_value), .seg7_enable(seg7_enable),
        .loop_tick(tick_strobe), .irq(irq_int), .fault_any(fault_any),
        .armed(armed),
        .m1_duty(m1_duty), .m2_duty(m2_duty), .duty_limit(duty_limit),
        .enc1_count(enc1_count), .enc2_count(enc2_count),
        .enc1_err(enc1_err), .enc2_err(enc2_err)
    );

    // ------------------------------------------------------------------
    // Motors: PWM outputs gated by ARMED, quadrature encoder inputs
    // ------------------------------------------------------------------
    hbridge_pwm #(.PERIOD(MAP_CLK_HZ / PWM_HZ)) u_m1 (
        .clk(clk), .rst(rst), .enable(armed),
        .duty(m1_duty), .limit(duty_limit), .in1(m1_in1), .in2(m1_in2)
    );
    hbridge_pwm #(.PERIOD(MAP_CLK_HZ / PWM_HZ)) u_m2 (
        .clk(clk), .rst(rst), .enable(armed),
        .duty(m2_duty), .limit(duty_limit), .in1(m2_in1), .in2(m2_in2)
    );

    quad_decoder #(.FILTER(ENC_FILTER_CLKS)) u_enc1 (
        .clk(clk), .rst(rst), .a_pin(enc1_a), .b_pin(enc1_b),
        .count(enc1_count), .err(enc1_err)
    );
    quad_decoder #(.FILTER(ENC_FILTER_CLKS)) u_enc2 (
        .clk(clk), .rst(rst), .a_pin(enc2_a), .b_pin(enc2_b),
        .count(enc2_count), .err(enc2_err)
    );

    // ------------------------------------------------------------------
    // LOOP_TICK: stretch the 1-clock strobe to a 1 us pulse
    // ------------------------------------------------------------------
    reg [6:0] tick_stretch = 7'd0;
    always @(posedge clk) begin
        if (rst)              tick_stretch <= 7'd0;
        else if (tick_strobe) tick_stretch <= 7'd100;
        else if (tick_stretch != 7'd0) tick_stretch <= tick_stretch - 7'd1;
    end
    assign loop_tick = (tick_stretch != 7'd0);

    assign fpga_irq = irq_int;

    // FAULT_N: low for ~0.1 s after any CRC error (visible on a scope or LED),
    // and held low while any safety fault is latched
    reg [23:0] fault_hold = 24'd0;
    always @(posedge clk) begin
        if (rst)          fault_hold <= 24'd0;
        else if (crc_err) fault_hold <= 24'd10_000_000;
        else if (fault_hold != 24'd0) fault_hold <= fault_hold - 24'd1;
    end
    assign fault_n = (fault_hold == 24'd0) && !fault_any;

    // ------------------------------------------------------------------
    // 7-segment display: 4 hex digits, ~1 kHz per digit refresh
    // ------------------------------------------------------------------
    reg [16:0] scan_div = 17'd0;
    reg [1:0]  digit = 2'd0;
    always @(posedge clk) begin
        if (scan_div == 17'd99_999) begin
            scan_div <= 17'd0;
            digit    <= digit + 2'd1;
        end else begin
            scan_div <= scan_div + 17'd1;
        end
    end

    reg [3:0] nibble;
    always @(*) begin
        case (digit)
            2'd0: nibble = seg7_value[3:0];
            2'd1: nibble = seg7_value[7:4];
            2'd2: nibble = seg7_value[11:8];
            default: nibble = seg7_value[15:12];
        endcase
    end

    reg [6:0] seg_n;   // active low, {g,f,e,d,c,b,a}
    always @(*) begin
        case (nibble)
            4'h0: seg_n = 7'b1000000;
            4'h1: seg_n = 7'b1111001;
            4'h2: seg_n = 7'b0100100;
            4'h3: seg_n = 7'b0110000;
            4'h4: seg_n = 7'b0011001;
            4'h5: seg_n = 7'b0010010;
            4'h6: seg_n = 7'b0000010;
            4'h7: seg_n = 7'b1111000;
            4'h8: seg_n = 7'b0000000;
            4'h9: seg_n = 7'b0010000;
            4'hA: seg_n = 7'b0001000;
            4'hB: seg_n = 7'b0000011;
            4'hC: seg_n = 7'b1000110;
            4'hD: seg_n = 7'b0100001;
            4'hE: seg_n = 7'b0000110;
            default: seg_n = 7'b0001110;
        endcase
    end

    // Link activity: hold ~50 ms after each good frame so it is visible
    reg [22:0] act_hold = 23'd0;
    always @(posedge clk) begin
        if (rst)                    act_hold <= 23'd0;
        else if (frame_ok)          act_hold <= 23'd5_000_000;
        else if (act_hold != 23'd0) act_hold <= act_hold - 23'd1;
    end

    assign seg = seg_n;
    // Decimal point of digit 0 blinks with link activity
    assign dp  = ~((act_hold != 23'd0) && digit == 2'd0);
    assign an  = ~(seg7_enable & (4'b0001 << digit));

endmodule
