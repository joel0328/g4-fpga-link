// regfile.v
// Register map for the MCU <-> FPGA link (map version 3: safety watchdog).
// Addresses are 7-bit; data is 32-bit. Keep fpga_regs.h in sync with this file.
//
// Addr  Name        Access  Description
// 0x00  ID          RO      0x4734_4650 ("G4FP") - link sanity check
// 0x01  VERSION     RO      Register map version (0x0000_0003)
// 0x02  SCRATCH     RW      Free read/write register for link tests
// 0x03  LEDS        RW      [15:0] Basys 3 LEDs (reads the commanded value;
//                           the pins are forced off unless SAFETY.OUT_EN)
// 0x04  SWITCHES    RO      [15:0] slide switches, [20:16] buttons U,L,R,D,C (debounced)
// 0x05  SEG7        RW      [15:0] hex value, [19:16] digit enables, [20] use_reg
//                           (use_reg=0: display shows link frame count)
// 0x06  TICK_DIV    RW      LOOP_TICK period in 100 MHz clocks (0 = off).
//                           100000 = 1 kHz. Values below 100 are clamped off.
// 0x07  TICK_COUNT  RO      Number of LOOP_TICKs since reset
// 0x08  IRQ_STATUS  RW1C    [0] tick, [1] switch/button change, [2] CRC error,
//                           [3] watchdog fault
// 0x09  IRQ_ENABLE  RW      Same bits; FPGA_IRQ = |(status & enable), active high
// 0x0A  FRAMES_OK   RO      Good frames received
// 0x0B  CRC_ERRS    RO      Frames rejected for bad CRC
// 0x0C  TIME_US     RO      Free-running microsecond timestamp
// 0x0D  CTRL        W       [0] write 1 to clear FRAMES_OK, CRC_ERRS, TICK_COUNT
// 0x0E  WDOG        RW      [23:0] watchdog timeout in us, 0 = off (reset value).
//                           Every good frame restarts the countdown.
// 0x0F  SAFETY      RW      [0] OUT_EN: actuator outputs enabled (LEDs for now).
//                           [1] FAULT: watchdog expired; latched, write 1 to clear.
//                           On expiry FAULT sets and OUT_EN clears. OUT_EN can't be
//                           set while FAULT is set (clear it in the same write).
// 0x10-0x1F SCRATCH_RAM RW  16 free words for burst tests
// 0x7F  (reserved)          burst escape in the frame header; never a register
//
// Unmapped addresses read 0xDEAD_BEEF.

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

    localparam [31:0] ID_VALUE = 32'h4734_4650;
    localparam [31:0] VERSION  = 32'h0000_0003;

    reg [31:0] scratch    = 32'h0;
    reg [15:0] leds_reg   = 16'h0;
    reg [20:0] seg7_reg   = 21'h0;
    reg [31:0] tick_div   = 32'h0;
    reg [31:0] tick_cnt   = 32'h0;
    reg [31:0] tick_count = 32'h0;
    reg [3:0]  irq_status = 4'b0;
    reg [3:0]  irq_enable = 4'b0;
    reg [31:0] frames_ok  = 32'h0;
    reg [31:0] crc_errs   = 32'h0;
    reg [31:0] time_us    = 32'h0;
    reg [6:0]  us_div     = 7'd0;
    reg [20:0] inputs_q   = 21'h0;
    reg [31:0] sram [0:15];          // SCRATCH_RAM, 0x10-0x1F (not reset)
    reg [23:0] wdog_timeout = 24'h0;
    reg [23:0] wdog_us      = 24'h0; // us since the last good frame (saturates)
    reg        out_en       = 1'b0;
    reg        fault        = 1'b0;

    wire [20:0] inputs_now = {buttons, switches};
    wire        sram_sel   = (addr[6:4] == 3'b001);

    integer k;
    initial for (k = 0; k < 16; k = k + 1) sram[k] = 32'h0;   // configuration value
    wire tick_on = (tick_div >= 32'd100);

    assign seg7_value  = seg7_reg[20] ? seg7_reg[15:0] : frames_ok[15:0];
    assign seg7_enable = seg7_reg[20] ? seg7_reg[19:16] : 4'hF;
    assign irq = |(irq_status & irq_enable);
    assign leds = out_en ? leds_reg : 16'h0;
    assign safety_fault = fault;

    wire us_tick   = (us_div == 7'd99);
    wire wdog_trip = (wdog_timeout != 24'h0) && (wdog_us >= wdog_timeout) && !fault;
    wire fault_clr = we && (addr == 7'h0F) && wdata[1];

    always @(*) begin
        if (sram_sel) rdata = sram[addr[3:0]];
        else case (addr)
            7'h00: rdata = ID_VALUE;
            7'h01: rdata = VERSION;
            7'h02: rdata = scratch;
            7'h03: rdata = {16'h0, leds_reg};
            7'h04: rdata = {11'h0, inputs_now};
            7'h05: rdata = {11'h0, seg7_reg};
            7'h06: rdata = tick_div;
            7'h07: rdata = tick_count;
            7'h08: rdata = {28'h0, irq_status};
            7'h09: rdata = {28'h0, irq_enable};
            7'h0A: rdata = frames_ok;
            7'h0B: rdata = crc_errs;
            7'h0C: rdata = time_us;
            7'h0D: rdata = 32'h0;
            7'h0E: rdata = {8'h0, wdog_timeout};
            7'h0F: rdata = {30'h0, fault, out_en};
            default: rdata = 32'hDEAD_BEEF;
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
            irq_status <= 4'b0;
            irq_enable <= 4'b0;
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
                    irq_status[0] <= 1'b1;
                end else begin
                    tick_cnt <= tick_cnt + 32'd1;
                end
            end else begin
                tick_cnt <= 32'h0;
            end

            // Input change detection
            inputs_q <= inputs_now;
            if (inputs_now != inputs_q)
                irq_status[1] <= 1'b1;

            // Link statistics
            if (frame_ok) frames_ok <= frames_ok + 32'd1;
            if (crc_err) begin
                crc_errs <= crc_errs + 32'd1;
                irq_status[2] <= 1'b1;
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
                irq_status[3] <= 1'b1;
            end

            // Register writes (applied after the event updates above, so a
            // write-1-to-clear in the same cycle as a new event clears it;
            // the event will simply reassert on its next occurrence)
            if (we && sram_sel)
                sram[addr[3:0]] <= wdata;

            if (we) begin
                case (addr)
                    7'h02: scratch  <= wdata;
                    7'h03: leds_reg <= wdata[15:0];
                    7'h05: seg7_reg <= wdata[20:0];
                    7'h06: tick_div <= wdata;
                    7'h08: irq_status <= irq_status & ~wdata[3:0];
                    7'h09: irq_enable <= wdata[3:0];
                    7'h0D: if (wdata[0]) begin
                               frames_ok  <= 32'h0;
                               crc_errs   <= 32'h0;
                               tick_count <= 32'h0;
                           end
                    7'h0E: wdog_timeout <= wdata[23:0];
                    7'h0F: begin
                               // FAULT is cleared by writing 1; OUT_EN only
                               // takes effect if FAULT is (now) clear. An
                               // expiry in the same cycle wins.
                               if (fault_clr && !wdog_trip) fault <= 1'b0;
                               out_en <= wdata[0] & ~(fault & ~fault_clr) & ~wdog_trip;
                           end
                    default: ;
                endcase
            end
        end
    end

endmodule
