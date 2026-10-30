// regfile.v
// Register file for the MCU <-> FPGA link. The map itself (addresses, access,
// fields, VERSION) lives in regs/regmap.toml; scripts/gen_regs.py turns it into
// fpga_regs.vh, which this file includes, so no address appears as a literal.
// The read mux and write logic stay hand-written.
//
// Behaviour notes (see README for the per-register table):
// - Safety layer (map v4). Outputs are live only while ARMED. Arming takes
//   ARM_KEY written to ARM with SW15 up, no fault latched and the tick running.
//   Any fault, SW15 down or another ARM write disarms at once. While armed the
//   MCU must write WDT_KICK; a tick that finds WDT_TICKS ticks since the last
//   kick latches FAULT.WDT. btnD latches FAULT.ESTOP at any time. Faults stay
//   latched until FAULT_CLEAR (ESTOP only once btnD is released).
// - TICK_DIV and WDT_TICKS can't change while armed, so the watchdog's time
//   base can't be stopped from software.
// - LED pins: [13:0] = LEDS while armed, else off; LED14 = fault, LED15 = armed.
// - Snapshot (SNAP_*): captured on every LOOP_TICK, published once CS is
//   idle, so a burst read never mixes two ticks.
// - TICK_DIV values below 100 turn LOOP_TICK off.
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
    input  wire        cs_active,    // a frame is in progress

    input  wire [15:0] switches,
    input  wire [4:0]  buttons,

    output wire [15:0] leds,         // [13:0] gated by ARMED, [14] fault, [15] armed
    output wire [15:0] seg7_value,
    output wire [3:0]  seg7_enable,

    output reg         loop_tick,
    output wire        irq,
    output wire        fault_any     // any fault latched
);

    `include "fpga_regs.vh"

    localparam FW = FAULT_STATUS_W;

    reg [31:0] scratch    = 32'h0;
    reg [LEDS_W-1:0] leds_reg = {LEDS_W{1'b0}};
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

    // Safety layer
    reg          armed     = 1'b0;
    reg [FW-1:0] fault_st  = {FW{1'b0}};
    reg [7:0]    wdt_ticks = WDT_TICKS_RESET[7:0];
    reg [7:0]    wdt_cnt   = 8'd0;   // ticks since the last kick (saturates)
    reg [2:0]    bad_run   = 3'd0;   // rejected frames in a row (saturates)

    // Snapshot: staged at the tick, published while CS is idle
    reg [31:0] snap_tick = 32'h0, snap_time = 32'h0, snap_inputs = 32'h0, snap_safety = 32'h0;
    reg [31:0] stg_tick  = 32'h0, stg_time  = 32'h0, stg_inputs  = 32'h0, stg_safety  = 32'h0;
    reg        snap_pend = 1'b0;

    wire [20:0] inputs_now = {buttons, switches};
    wire [6:0]  sram_off   = addr - REG_SCRATCH_RAM;
    wire        sram_sel   = (addr >= REG_SCRATCH_RAM) &&
                             (addr <  REG_SCRATCH_RAM + REG_SCRATCH_RAM_WORDS);

    integer k;
    initial for (k = 0; k < REG_SCRATCH_RAM_WORDS; k = k + 1) sram[k] = 32'h0;   // configuration value
    wire tick_on   = (tick_div >= 32'd100);
    wire tick_fire = tick_on && (tick_cnt >= tick_div - 32'd1);
    wire us_tick   = (us_div == 7'd99);

    assign seg7_value  = seg7_reg[SEG7_USE_REG] ? seg7_reg[SEG7_HEX_MSB:SEG7_HEX_LSB] : frames_ok[15:0];
    assign seg7_enable = seg7_reg[SEG7_USE_REG] ? seg7_reg[SEG7_DIGITS_MSB:SEG7_DIGITS_LSB] : 4'hF;
    assign irq = |(irq_status & irq_enable);
    assign fault_any = |fault_st;
    assign leds = {armed, fault_any, armed ? leds_reg : {LEDS_W{1'b0}}};

    // ---- Safety decisions for this clock
    wire arm_en  = inputs_now[ARM_SWITCH];
    wire estop   = inputs_now[BTN_D];
    wire kick    = we && (addr == REG_WDT_KICK);
    wire arm_req = we && (addr == REG_ARM);

    reg [FW-1:0] f_new, f_clr;
    always @(*) begin
        f_new = {FW{1'b0}};
        f_new[FAULT_WDT]      = armed && tick_fire && !kick && (wdt_cnt >= wdt_ticks);
        f_new[FAULT_LINK]     = armed && crc_err && (bad_run >= LINK_FAULT_ERRS - 1);
        f_new[FAULT_ESTOP]    = estop;
        f_new[FAULT_ARM_LOST] = armed && !arm_en;
        // ESTOP is level-sensitive: while btnD is held it re-latches on the
        // same clock as any FAULT_CLEAR, so it can't be cleared until released
        f_clr = (we && addr == REG_FAULT_CLEAR) ? wdata[FW-1:0] : {FW{1'b0}};
    end

    wire arm_ok = arm_req && (wdata == ARM_KEY) && arm_en && tick_on &&
                  !fault_any && (f_new == {FW{1'b0}});
    wire disarm = arm_req || fault_any || (f_new != {FW{1'b0}}) || !arm_en;

    reg [31:0] arm_rd;
    always @(*) begin
        arm_rd = 32'h0;
        arm_rd[ARM_ARMED]   = armed;
        arm_rd[ARM_SW_EN]   = arm_en;
        arm_rd[ARM_FAULT]   = fault_any;
        arm_rd[ARM_TICK_ON] = tick_on;
    end

    always @(*) begin
        if (sram_sel) rdata = sram[sram_off];
        else case (addr)
            REG_ID:           rdata = MAP_ID;
            REG_VERSION:      rdata = MAP_VERSION;
            REG_SCRATCH:      rdata = scratch;
            REG_LEDS:         rdata = {{(32-LEDS_W){1'b0}}, leds_reg};
            REG_SWITCHES:     rdata = {11'h0, inputs_now};
            REG_SEG7:         rdata = {11'h0, seg7_reg};
            REG_TICK_DIV:     rdata = tick_div;
            REG_TICK_COUNT:   rdata = tick_count;
            REG_IRQ_STATUS:   rdata = {{(32-IRQ_STATUS_W){1'b0}}, irq_status};
            REG_IRQ_ENABLE:   rdata = {{(32-IRQ_STATUS_W){1'b0}}, irq_enable};
            REG_FRAMES_OK:    rdata = frames_ok;
            REG_CRC_ERRS:     rdata = crc_errs;
            REG_TIME_US:      rdata = time_us;
            REG_CTRL:         rdata = 32'h0;
            REG_ARM:          rdata = arm_rd;
            REG_WDT_TICKS:    rdata = {24'h0, wdt_ticks};
            REG_FAULT_STATUS: rdata = {{(32-FW){1'b0}}, fault_st};
            REG_FAULT_CLEAR:  rdata = 32'h0;
            REG_WDT_KICK:     rdata = {24'h0, wdt_cnt};
            REG_SNAP_TICK:    rdata = snap_tick;
            REG_SNAP_TIME_US: rdata = snap_time;
            REG_SNAP_INPUTS:  rdata = snap_inputs;
            REG_SNAP_SAFETY:  rdata = snap_safety;
            default:          rdata = MAP_UNMAPPED;
        endcase
    end

    always @(posedge clk) begin
        loop_tick <= 1'b0;

        if (rst) begin
            scratch    <= 32'h0;
            leds_reg   <= {LEDS_W{1'b0}};
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
            armed      <= 1'b0;
            fault_st   <= {FW{1'b0}};
            wdt_ticks  <= WDT_TICKS_RESET[7:0];
            wdt_cnt    <= 8'd0;
            bad_run    <= 3'd0;
            snap_tick  <= 32'h0;
            snap_time  <= 32'h0;
            snap_inputs <= 32'h0;
            snap_safety <= 32'h0;
            snap_pend  <= 1'b0;
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
                if (tick_fire) begin
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

            // Snapshot: stage at the tick, publish when no frame is running
            if (tick_fire) begin
                stg_tick    <= tick_count + 32'd1;
                stg_time    <= time_us;
                stg_inputs  <= {11'h0, inputs_now};
                stg_safety  <= {{(32-SNAP_FAULT_LSB-FW){1'b0}}, fault_st,
                                {(SNAP_FAULT_LSB-4){1'b0}}, arm_rd[3:0]};
                snap_pend   <= 1'b1;
            end else if (snap_pend && !cs_active) begin
                snap_tick   <= stg_tick;
                snap_time   <= stg_time;
                snap_inputs <= stg_inputs;
                snap_safety <= stg_safety;
                snap_pend   <= 1'b0;
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
            if (frame_ok)
                bad_run <= 3'd0;
            else if (crc_err && bad_run != 3'd7)
                bad_run <= bad_run + 3'd1;

            // Safety layer: faults latch (a new cause beats a clear in the
            // same clock); arming only from a clean state
            fault_st <= (fault_st & ~f_clr) | f_new;
            if (|(f_new & ~fault_st))
                irq_status[IRQ_FAULT] <= 1'b1;
            if (arm_ok)
                armed <= 1'b1;
            else if (disarm)
                armed <= 1'b0;
            if (!armed || kick)
                wdt_cnt <= 8'd0;
            else if (tick_fire && wdt_cnt != 8'hFF)
                wdt_cnt <= wdt_cnt + 8'd1;

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
                    REG_TICK_DIV:   if (!armed) tick_div <= wdata;
                    REG_IRQ_STATUS: irq_status <= irq_status & ~wdata[IRQ_STATUS_W-1:0];
                    REG_IRQ_ENABLE: irq_enable <= wdata[IRQ_STATUS_W-1:0];
                    REG_CTRL:       if (wdata[CTRL_CLEAR_COUNTERS]) begin
                                        frames_ok  <= 32'h0;
                                        crc_errs   <= 32'h0;
                                        tick_count <= 32'h0;
                                    end
                    REG_WDT_TICKS:  if (!armed)
                                        wdt_ticks <= (wdata[7:0] == 8'd0) ? 8'd1 : wdata[7:0];
                    default: ;
                endcase
            end
        end
    end

endmodule
