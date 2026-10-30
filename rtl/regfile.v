// regfile.v
// Register file for the MCU <-> FPGA link. The map itself (addresses, access,
// fields, VERSION) lives in regs/regmap.toml; scripts/gen_regs.py turns it into
// fpga_regs.vh, which this file includes, so no address appears as a literal.
// The read mux and write logic stay hand-written.
//
// Behaviour notes (see README for the per-register table):
// - LEDS reads the commanded value; the pins are forced off unless SAFETY.OUT_EN.
// - TICK_DIV values below 100 turn LOOP_TICK off.
// - WDOG: every good frame restarts the countdown. On expiry SAFETY.FAULT
//   latches and OUT_EN clears. OUT_EN can't be set while FAULT is set (clear
//   FAULT in the same write).
// - SCRATCH_RAM is a block of words for burst tests (not reset).
// - Unmapped addresses read MAP_UNMAPPED (0xDEAD_BEEF).

module regfile (
    input  wire        clk,
    input  wire        rst,

    input  wire [6:0]  addr,
    input  wire [31:0] wdata,
    input  wire        we,
    output reg  [31:0] rdata,

    input  wire        frame_ok,
    input  wire        crc_err,

    input  wire [15:0] switches,
    input  wire [4:0]  buttons,

    output wire [15:0] leds,         // gated by SAFETY.OUT_EN
    output wire [15:0] seg7_value,
    output wire [3:0]  seg7_enable,

    output reg         loop_tick,
    output wire        irq,
    output wire        safety_fault
);

    `include "fpga_regs.vh"

    reg [31:0] scratch    = 32'h0;
    reg [15:0] leds_reg   = 16'h0;
    reg [20:0] seg7_reg   = 21'h0;
    reg [31:0] tick_div   = 32'h0;
    reg [31:0] tick_cnt   = 32'h0;
    reg [31:0] tick_count = 32'h0;
    reg [IRQ_STATUS_W-1:0] irq_status = {IRQ_STATUS_W{1'b0}};
    reg [IRQ_STATUS_W-1:0] irq_enable = {IRQ_STATUS_W{1'b0}};
    reg [31:0] frames_ok  = 32'h0;
    reg [31:0] crc_errs   = 32'h0;
    reg [31:0] time_us    = 32'h0;
    reg [6:0]  us_div     = 7'd0;
    reg [20:0] inputs_q   = 21'h0;
    reg [31:0] sram [0:REG_SCRATCH_RAM_WORDS-1];   // not reset
    reg [23:0] wdog_timeout = 24'h0;
    reg [23:0] wdog_us      = 24'h0; // us since the last good frame (saturates)
    reg        out_en       = 1'b0;
    reg        fault        = 1'b0;

    wire [20:0] inputs_now = {buttons, switches};
    wire [6:0]  sram_off   = addr - REG_SCRATCH_RAM;
    wire        sram_sel   = (addr >= REG_SCRATCH_RAM) &&
                             (addr <  REG_SCRATCH_RAM + REG_SCRATCH_RAM_WORDS);

    integer k;
    initial for (k = 0; k < REG_SCRATCH_RAM_WORDS; k = k + 1) sram[k] = 32'h0;   // configuration value
    wire tick_on = (tick_div >= 32'd100);

    assign seg7_value  = seg7_reg[SEG7_USE_REG] ? seg7_reg[SEG7_HEX_MSB:SEG7_HEX_LSB] : frames_ok[15:0];
    assign seg7_enable = seg7_reg[SEG7_USE_REG] ? seg7_reg[SEG7_DIGITS_MSB:SEG7_DIGITS_LSB] : 4'hF;
    assign irq = |(irq_status & irq_enable);
    assign leds = out_en ? leds_reg : 16'h0;
    assign safety_fault = fault;

    wire us_tick   = (us_div == 7'd99);
    wire wdog_trip = (wdog_timeout != 24'h0) && (wdog_us >= wdog_timeout) && !fault;
    wire fault_clr = we && (addr == REG_SAFETY) && wdata[SAFETY_FAULT];

    always @(*) begin
        if (sram_sel) rdata = sram[sram_off];
        else case (addr)
            REG_ID:         rdata = MAP_ID;
            REG_VERSION:    rdata = MAP_VERSION;
            REG_SCRATCH:    rdata = scratch;
            REG_LEDS:       rdata = {16'h0, leds_reg};
            REG_SWITCHES:   rdata = {11'h0, inputs_now};
            REG_SEG7:       rdata = {11'h0, seg7_reg};
            REG_TICK_DIV:   rdata = tick_div;
            REG_TICK_COUNT: rdata = tick_count;
            REG_IRQ_STATUS: rdata = {{(32-IRQ_STATUS_W){1'b0}}, irq_status};
            REG_IRQ_ENABLE: rdata = {{(32-IRQ_STATUS_W){1'b0}}, irq_enable};
            REG_FRAMES_OK:  rdata = frames_ok;
            REG_CRC_ERRS:   rdata = crc_errs;
            REG_TIME_US:    rdata = time_us;
            REG_CTRL:       rdata = 32'h0;
            REG_WDOG:       rdata = {8'h0, wdog_timeout};
            REG_SAFETY:     rdata = {30'h0, fault, out_en};
            default:        rdata = MAP_UNMAPPED;
        endcase
    end

    always @(posedge clk) begin
        loop_tick <= 1'b0;

        if (rst) begin
            scratch    <= 32'h0;
            leds_reg   <= 16'h0;
            seg7_reg   <= 21'h0;
            tick_div   <= 32'h0;
            tick_cnt   <= 32'h0;
            tick_count <= 32'h0;
            irq_status <= {IRQ_STATUS_W{1'b0}};
            irq_enable <= {IRQ_STATUS_W{1'b0}};
            frames_ok  <= 32'h0;
            crc_errs   <= 32'h0;
            time_us    <= 32'h0;
            us_div     <= 7'd0;
            inputs_q   <= inputs_now;
            wdog_timeout <= 24'h0;
            wdog_us    <= 24'h0;
            out_en     <= 1'b0;
            fault      <= 1'b0;
        end else begin
            // Microsecond timestamp
            if (us_tick) begin
                us_div  <= 7'd0;
                time_us <= time_us + 32'd1;
            end else begin
                us_div <= us_div + 7'd1;
            end

            // Loop tick: one-clock strobe; the top stretches it into a visible pulse
            if (tick_on) begin
                if (tick_cnt >= tick_div - 32'd1) begin
                    tick_cnt   <= 32'h0;
                    loop_tick  <= 1'b1;
                    tick_count <= tick_count + 32'd1;
                    irq_status[IRQ_TICK] <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 32'd1;
                end
            end else begin
                tick_cnt <= 32'h0;
            end

            // Input change detection
            inputs_q <= inputs_now;
            if (inputs_now != inputs_q)
                irq_status[IRQ_INPUTS] <= 1'b1;

            // Link statistics
            if (frame_ok) frames_ok <= frames_ok + 32'd1;
            if (crc_err) begin
                crc_errs <= crc_errs + 32'd1;
                irq_status[IRQ_CRC_ERR] <= 1'b1;
            end

            // Watchdog: count microseconds since the last good frame. On
            // expiry latch FAULT and drop the outputs; only the MCU can
            // clear FAULT and re-enable them.
            if (frame_ok)
                wdog_us <= 24'h0;
            else if (us_tick && wdog_us != 24'hFF_FFFF)
                wdog_us <= wdog_us + 24'd1;
            if (wdog_trip) begin
                fault         <= 1'b1;
                out_en        <= 1'b0;
                irq_status[IRQ_WDOG] <= 1'b1;
            end

            // Register writes (applied after the event updates above, so a
            // write-1-to-clear in the same cycle as a new event clears it;
            // the event will simply reassert on its next occurrence)
            if (we && sram_sel)
                sram[sram_off] <= wdata;

            if (we) begin
                case (addr)
                    REG_SCRATCH:    scratch  <= wdata;
                    REG_LEDS:       leds_reg <= wdata[LEDS_VALUE_MSB:LEDS_VALUE_LSB];
                    REG_SEG7:       seg7_reg <= wdata[SEG7_W-1:0];
                    REG_TICK_DIV:   tick_div <= wdata;
                    REG_IRQ_STATUS: irq_status <= irq_status & ~wdata[IRQ_STATUS_W-1:0];
                    REG_IRQ_ENABLE: irq_enable <= wdata[IRQ_STATUS_W-1:0];
                    REG_CTRL:       if (wdata[CTRL_CLEAR_COUNTERS]) begin
                               frames_ok  <= 32'h0;
                               crc_errs   <= 32'h0;
                               tick_count <= 32'h0;
                           end
                    REG_WDOG:       wdog_timeout <= wdata[WDOG_TIMEOUT_MSB:WDOG_TIMEOUT_LSB];
                    REG_SAFETY:     begin
                               // FAULT is cleared by writing 1; OUT_EN only
                               // takes effect if FAULT is (now) clear. An
                               // expiry in the same cycle wins.
                               if (fault_clr && !wdog_trip) fault <= 1'b0;
                               out_en <= wdata[SAFETY_OUT_EN] & ~(fault & ~fault_clr) & ~wdog_trip;
                           end
                    default: ;
                endcase
            end
        end
    end

endmodule
