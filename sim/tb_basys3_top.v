// tb_basys3_top.v
// Self-checking testbench for the SPI register link. Models the STM32 as a
// mode-0 SPI master at several clock rates, with a configurable round-trip
// delay on MISO to approximate board wiring and I/O buffers.
//
// Run:  iverilog -g2012 -o tb.vvp tb_basys3_top.v ../rtl/*.v && vvp tb.vvp

`timescale 1ns/1ps

module tb_basys3_top;

    reg clk = 1'b0;
    always #5 clk = ~clk;   // 100 MHz

    reg  [15:0] sw = 16'h8000;   // SW15 up: hardware arm enable
    reg  btnC = 0, btnU = 0, btnL = 0, btnR = 0, btnD = 0;
    wire [15:0] led;
    wire [6:0]  seg;
    wire        dp;
    wire [3:0]  an;

    reg  sck = 1'b0, mosi = 1'b0, cs_n = 1'b1, rst_n = 1'b1;
    wire miso_pin, irq, tick, fault_n;

    // Wiring + buffer delay seen by the master when sampling MISO
    real miso_delay_ns = 8.0;
    reg  miso_seen = 1'b0;   // MISO idles low (tx_shift resets to 0)
    always @(miso_pin) miso_seen <= #(miso_delay_ns) miso_pin;

    // Hold check: MISO must not change within 15 ns after an SCK rising edge
    // (the STM32 samples there; datasheet hold is a few ns).
    realtime last_rise = 0;
    integer hold_violations = 0;
    always @(posedge sck) last_rise = $realtime;
    always @(miso_pin)
        if (!cs_n && sck && ($realtime - last_rise) < 15.0)
            hold_violations = hold_violations + 1;

    basys3_top dut (
        .clk(clk), .sw(sw),
        .btnC(btnC), .btnU(btnU), .btnL(btnL), .btnR(btnR), .btnD(btnD),
        .led(led), .seg(seg), .dp(dp), .an(an),
        .spi_sck(sck), .spi_mosi(mosi), .spi_miso(miso_pin), .spi_cs_n(cs_n),
        .fpga_irq(irq), .loop_tick(tick), .fpga_rst_n(rst_n), .fault_n(fault_n)
    );

    // ------------------------------------------------------------------
    // Helpers
    // ------------------------------------------------------------------
    integer errors = 0;
    integer checks = 0;
    real    half_ns = 47.0;   // SPI half period (10.6 MHz default)

    function [7:0] crc8;
        input [7:0] crc_in;
        input [7:0] data;
        integer i;
        reg [7:0] x;
        begin
            x = crc_in ^ data;
            for (i = 0; i < 8; i = i + 1)
                x = x[7] ? ((x << 1) ^ 8'h07) : (x << 1);
            crc8 = x;
        end
    endfunction

    task check(input [255:0] what, input [31:0] got, input [31:0] exp);
        begin
            checks = checks + 1;
            if (got !== exp) begin
                errors = errors + 1;
                $display("  FAIL %0s: got %08h expected %08h", what, got, exp);
            end
        end
    endtask

    task spi_byte(input [7:0] tx, output [7:0] rx);
        integer b;
        begin
            for (b = 7; b >= 0; b = b - 1) begin
                mosi = tx[b];
                #(half_ns);
                sck = 1'b1;
                rx[b] = miso_seen;
                #(half_ns);
                sck = 1'b0;
            end
        end
    endtask

    task cs_begin;
        begin
            cs_n = 1'b0;
            #(half_ns);
        end
    endtask

    task cs_end;
        begin
            #(half_ns);
            cs_n = 1'b1;
            #(200);   // inter-frame gap (MCU software overhead is far larger)
        end
    endtask

    task reg_write(input [6:0] addr, input [31:0] data, input bad_crc);
        reg [7:0] crc, rx;
        begin
            crc = crc8(8'h00, {1'b0, addr});
            crc = crc8(crc, data[31:24]);
            crc = crc8(crc, data[23:16]);
            crc = crc8(crc, data[15:8]);
            crc = crc8(crc, data[7:0]);
            if (bad_crc) crc = crc ^ 8'h01;
            cs_begin;
            spi_byte({1'b0, addr}, rx);
            spi_byte(data[31:24], rx);
            spi_byte(data[23:16], rx);
            spi_byte(data[15:8], rx);
            spi_byte(data[7:0], rx);
            spi_byte(crc, rx);
            cs_end;
        end
    endtask

    // Returns data and whether the response CRC checked out
    task reg_read(input [6:0] addr, input bad_hdr_crc,
                  output [31:0] data, output crc_ok);
        reg [7:0] hdr_crc, rx, rx_crc, calc;
        begin
            hdr_crc = crc8(8'h00, {1'b1, addr});
            if (bad_hdr_crc) hdr_crc = hdr_crc ^ 8'h80;
            cs_begin;
            spi_byte({1'b1, addr}, rx);
            spi_byte(hdr_crc, rx);
            spi_byte(8'h00, data[31:24]);
            spi_byte(8'h00, data[23:16]);
            spi_byte(8'h00, data[15:8]);
            spi_byte(8'h00, data[7:0]);
            spi_byte(8'h00, rx_crc);
            cs_end;
            calc = crc8(crc8(crc8(crc8(8'h00, data[31:24]), data[23:16]),
                             data[15:8]), data[7:0]);
            crc_ok = (calc == rx_crc);
        end
    endtask

    task read_expect(input [255:0] what, input [6:0] addr, input [31:0] exp);
        reg [31:0] d;
        reg ok;
        begin
            reg_read(addr, 1'b0, d, ok);
            check({what, " (crc)"}, {31'h0, ok}, 32'h1);
            check(what, d, exp);
        end
    endtask

    // ------------------------------------------------------------------
    // Burst frames (map version 2)
    // ------------------------------------------------------------------
    reg [31:0] bw   [0:15];   // words to burst-write
    reg [31:0] br   [0:15];   // words from the last burst read
    reg [31:0] bref [0:15];   // saved copy for "unchanged" checks

    // Burst write of n words from bw[]. cnt_byte is sent as N-1 (pass n-1
    // unless testing rejection). cut_after > 0 releases CS once that many
    // bytes (header included) have gone out.
    task burst_write(input [6:0] start, input integer n, input [7:0] cnt_byte,
                     input bad_hdr, input bad_data, input integer cut_after);
        reg [7:0] crc, rx, b;
        integer w, j, sent;
        begin
            crc = crc8(crc8(crc8(8'h00, 8'h7F), {1'b0, start}), cnt_byte);
            if (bad_hdr) crc = crc ^ 8'h01;
            cs_begin;
            spi_byte(8'h7F, rx);
            spi_byte({1'b0, start}, rx);
            spi_byte(cnt_byte, rx);
            spi_byte(crc, rx);
            sent = 4;
            crc = 8'h00;
            for (w = 0; w < n; w = w + 1)
                for (j = 3; j >= 0; j = j - 1) begin
                    b = bw[w] >> (8 * j);
                    crc = crc8(crc, b);
                    if (cut_after == 0 || sent < cut_after) spi_byte(b, rx);
                    sent = sent + 1;
                end
            if (bad_data) crc = crc ^ 8'h01;
            if (cut_after == 0 || sent < cut_after) spi_byte(crc, rx);
            cs_end;
        end
    endtask

    // Burst read of n words into br[]; crc_ok says whether the data CRC matched
    task burst_read(input [6:0] start, input integer n, input [7:0] cnt_byte,
                    input bad_hdr, output crc_ok);
        reg [7:0] crc, rx, calc;
        reg [31:0] word;
        integer w, j;
        begin
            crc = crc8(crc8(crc8(8'h00, 8'hFF), {1'b0, start}), cnt_byte);
            if (bad_hdr) crc = crc ^ 8'h80;
            cs_begin;
            spi_byte(8'hFF, rx);
            spi_byte({1'b0, start}, rx);
            spi_byte(cnt_byte, rx);
            spi_byte(crc, rx);
            calc = 8'h00;
            for (w = 0; w < n; w = w + 1) begin
                for (j = 0; j < 4; j = j + 1) begin
                    spi_byte(8'h00, rx);
                    word = {word[23:0], rx};
                    calc = crc8(calc, rx);
                end
                br[w] = word;
            end
            spi_byte(8'h00, rx);
            crc_ok = (rx == calc);
            cs_end;
        end
    endtask

    task burst_suite(input [31:0] seed);
        reg [31:0] d, f0;
        reg ok;
        integer i;
        begin
            // 16 words through SCRATCH_RAM
            for (i = 0; i < 16; i = i + 1)
                bw[i] = (32'h9E37_79B9 * (i + 1)) ^ seed;
            reg_read(7'h0A, 1'b0, f0, ok);
            burst_write(7'h10, 16, 8'd15, 1'b0, 1'b0, 0);
            burst_read(7'h10, 16, 8'd15, 1'b0, ok);
            check("burst read x16 (crc)", {31'h0, ok}, 32'h1);
            for (i = 0; i < 16; i = i + 1)
                check("burst RAM word", br[i], bw[i]);
            read_expect("RAM[0] by single read", 7'h10, bw[0]);
            read_expect("RAM[15] by single read", 7'h1F, bw[15]);
            reg_read(7'h0A, 1'b0, d, ok);
            // f0's own read, burst write, burst read, two single reads
            check("FRAMES_OK counts a burst as one frame", d - f0, 32'd5);

            // Burst across ordinary registers: SCRATCH then LEDS
            bw[0] = 32'h0BAD_CAFE ^ seed;
            bw[1] = 32'h0000_3C3C;
            burst_write(7'h02, 2, 8'd1, 1'b0, 1'b0, 0);
            #20;
            check("LED pins by burst write", {16'h0, led}, 32'h0000_BC3C);   // LED15 = armed
            read_expect("SCRATCH by burst write", 7'h02, bw[0]);

            // N = 1
            burst_read(7'h02, 1, 8'd0, 1'b0, ok);
            check("burst read x1 (crc)", {31'h0, ok}, 32'h1);
            check("burst read x1", br[0], bw[0]);

            // ID + VERSION in one frame
            burst_read(7'h00, 2, 8'd1, 1'b0, ok);
            check("burst ID+VERSION (crc)", {31'h0, ok}, 32'h1);
            check("burst ID", br[0], 32'h4734_4650);
            check("burst VERSION", br[1], 32'h0000_0004);

            // Addresses wrap: 0x7E, 0x7F, 0x00
            burst_read(7'h7E, 3, 8'd2, 1'b0, ok);
            check("burst wrap (crc)", {31'h0, ok}, 32'h1);
            check("burst wrap 0x7E", br[0], 32'hDEAD_BEEF);
            check("burst wrap 0x7F", br[1], 32'hDEAD_BEEF);
            check("burst wrap 0x00", br[2], 32'h4734_4650);
        end
    endtask

    // ------------------------------------------------------------------
    // Link test at one SPI speed
    // ------------------------------------------------------------------
    task link_suite(input real hp, input real delay);
        reg [31:0] d, pattern, frames_before;
        reg ok;
        integer i;
        begin
            half_ns = hp;
            miso_delay_ns = delay;
            $display("SPI %.1f MHz, MISO delay %.0f ns", 1000.0 / (2.0 * hp), delay);

            read_expect("ID", 7'h00, 32'h4734_4650);
            read_expect("VERSION", 7'h01, 32'h0000_0004);

            // Scratch: walking patterns
            for (i = 0; i < 32; i = i + 1) begin
                pattern = (32'h1 << i) ^ {i[7:0], i[7:0], i[7:0], i[7:0]};
                reg_write(7'h02, pattern, 1'b0);
                read_expect("SCRATCH walk", 7'h02, pattern);
            end
            reg_write(7'h02, 32'hFFFF_FFFF, 1'b0);
            read_expect("SCRATCH ones", 7'h02, 32'hFFFF_FFFF);
            reg_write(7'h02, 32'h0000_0000, 1'b0);
            read_expect("SCRATCH zeros", 7'h02, 32'h0000_0000);

            // LEDs drive the pins: [13:0] from LEDS, LED15 = armed, LED14 = fault
            reg_write(7'h03, 32'h0000_E5C3, 1'b0);
            #20;
            check("LED pins", {16'h0, led}, 32'h0000_A5C3);
            read_expect("LEDS (14 bits)", 7'h03, 32'h0000_25C3);

            // Unmapped address (0x7F is the burst escape, so use 0x7E)
            read_expect("unmapped", 7'h7E, 32'hDEAD_BEEF);

            // Frame counter increases by exactly the number of good frames
            reg_read(7'h0A, 1'b0, frames_before, ok);
            reg_write(7'h02, 32'h1234_5678, 1'b0);
            reg_write(7'h02, 32'h8765_4321, 1'b0);
            reg_read(7'h0A, 1'b0, d, ok);
            // the first read itself, two writes, and this read's own frame
            // completes after its data is latched, so it is not counted yet
            check("FRAMES_OK delta", d - frames_before, 32'd3);

            burst_suite(32'hC0FF_EE00 ^ $rtoi(hp * 1000.0));   // new data per speed
        end
    endtask

    // ------------------------------------------------------------------
    // Safety layer helpers (map v4)
    // ------------------------------------------------------------------
    localparam [31:0] ARM_KEY = 32'h4152_4D21;

    task wait_inputs;   // switches/buttons are sampled every 1 ms
        #(2_100_000);
    endtask

    task arm;
        reg [31:0] d;
        reg ok;
        begin
            reg_write(7'h20, ARM_KEY, 1'b0);
            reg_read(7'h20, 1'b0, d, ok);
            check("armed", d & 32'h1, 32'h1);
        end
    endtask

    task snap_read(output [31:0] s0, output [31:0] s1, output [31:0] s2, output [31:0] s3);
        reg ok;
        begin
            burst_read(7'h30, 4, 8'd3, 1'b0, ok);
            check("snapshot burst (crc)", {31'h0, ok}, 32'h1);
            s0 = br[0]; s1 = br[1]; s2 = br[2]; s3 = br[3];
        end
    endtask

    // ------------------------------------------------------------------
    // Main sequence
    // ------------------------------------------------------------------
    reg [31:0] d, prev_cnt, t0, t1;
    reg [31:0] s_a [0:3];
    reg [31:0] s_b [0:3];
    reg [31:0] s_c [0:3];
    reg ok;
    integer pulses;
    integer pulse_width;
    integer i;

    always @(posedge tick) pulses = pulses + 1;

    // Safety invariant on every clock: never armed while a fault is latched
    // (a one-clock arm would be a glitch on every actuator output)
    integer armed_with_fault = 0;
    always @(posedge clk)
        if (dut.u_regs.armed && dut.fault_any)
            armed_with_fault = armed_with_fault + 1;

    initial begin
        pulses = 0;
        $display("=== SPI link testbench ===");
        wait_inputs;   // past power-on reset, SW15 sampled

        // Outputs (LED pins) stay off until armed. Arm for the link tests
        // with a 100 ms tick, so the watchdog never comes due during them.
        check("LED pins off before arming", {16'h0, led}, 32'h0);
        reg_write(7'h06, 32'd10_000_000, 1'b0);
        arm;

        // Speeds: 5.3 MHz (prescaler /32), 10.6 MHz (/16), 12.5 MHz stress
        link_suite(94.0, 8.0);
        link_suite(47.0, 8.0);
        link_suite(40.0, 5.0);

        half_ns = 47.0;
        miso_delay_ns = 8.0;
        $display("Error handling, IRQ, tick, reset");

        // Write with bad CRC must be ignored and counted
        reg_write(7'h02, 32'hCAFE_F00D, 1'b0);
        reg_read(7'h0B, 1'b0, prev_cnt, ok);
        reg_write(7'h02, 32'h0BAD_0BAD, 1'b1);
        #20;
        check("FAULT_N low after CRC error", {31'h0, fault_n}, 32'h0);
        read_expect("SCRATCH unchanged by bad write", 7'h02, 32'hCAFE_F00D);
        reg_read(7'h0B, 1'b0, d, ok);
        check("CRC_ERRS +1 after bad write", d - prev_cnt, 32'd1);

        // Read with bad header CRC returns all ones with an inverted CRC
        reg_read(7'h00, 1'b1, d, ok);
        check("bad-header read data", d, 32'hFFFF_FFFF);
        check("bad-header read flagged", {31'h0, ok}, 32'h0);

        // Aborted write (CS released after 3 bytes) must not write
        begin : aborted
            reg [7:0] rx;
            cs_begin;
            spi_byte({1'b0, 7'h02}, rx);
            spi_byte(8'h11, rx);
            spi_byte(8'h22, rx);
            cs_end;
        end
        read_expect("SCRATCH unchanged by aborted frame", 7'h02, 32'hCAFE_F00D);

        // A good frame still works right after the errors
        reg_write(7'h02, 32'h600D_600D, 1'b0);
        read_expect("SCRATCH after recovery", 7'h02, 32'h600D_600D);

        // Burst errors: nothing may be written, each rejected frame counts once
        $display("Burst error handling");
        for (i = 0; i < 16; i = i + 1) begin
            bw[i]   = 32'h1111_0000 + i;
            bref[i] = bw[i];
        end
        burst_write(7'h10, 16, 8'd15, 1'b0, 1'b0, 0);   // known RAM contents
        reg_read(7'h0B, 1'b0, prev_cnt, ok);
        for (i = 0; i < 16; i = i + 1)
            bw[i] = 32'hBAD0_0000 + i;
        burst_write(7'h10, 16, 8'd15, 1'b0, 1'b1, 0);   // bad data CRC
        burst_write(7'h10, 16, 8'd15, 1'b1, 1'b0, 0);   // bad header CRC
        burst_write(7'h10, 16, 8'h10, 1'b0, 1'b0, 0);   // N = 17
        burst_write(7'h10, 16, 8'd15, 1'b0, 1'b0, 20);  // cut mid-data
        burst_write(7'h10, 16, 8'd15, 1'b0, 1'b0, 68);  // cut before the data CRC
        reg_read(7'h0B, 1'b0, d, ok);
        check("CRC_ERRS +3 after bad burst writes", d - prev_cnt, 32'd3);
        burst_read(7'h10, 16, 8'd15, 1'b0, ok);
        check("RAM readback after bad bursts (crc)", {31'h0, ok}, 32'h1);
        for (i = 0; i < 16; i = i + 1)
            check("RAM unchanged by rejected bursts", br[i], bref[i]);

        reg_read(7'h0B, 1'b0, prev_cnt, ok);
        burst_read(7'h10, 4, 8'd3, 1'b1, ok);            // bad header CRC
        check("bad-header burst read flagged", {31'h0, ok}, 32'h0);
        check("bad-header burst read data", br[0], 32'hFFFF_FFFF);
        burst_read(7'h10, 1, 8'h10, 1'b0, ok);           // N = 17
        check("N=17 burst read flagged", {31'h0, ok}, 32'h0);
        check("N=17 burst read data", br[0], 32'hFFFF_FFFF);
        reg_read(7'h0B, 1'b0, d, ok);
        check("CRC_ERRS +2 after bad burst reads", d - prev_cnt, 32'd2);

        check("3 rejected frames in a row don't trip LINK", {31'h0, dut.fault_any}, 32'h0);
        read_expect("still armed after error tests", 7'h20, 32'hB);

        // Single frames still work right after a burst
        burst_write(7'h10, 1, 8'd0, 1'b0, 1'b0, 0);
        reg_write(7'h02, 32'h5EED_5EED, 1'b0);
        read_expect("single write right after a burst", 7'h02, 32'h5EED_5EED);

        // Safety layer (map v4)
        $display("Safety layer");
        reg_write(7'h03, 32'h0000_00FF, 1'b0);
        read_expect("ARM: armed, SW15, tick on", 7'h20, 32'hB);
        read_expect("FAULT_STATUS clear", 7'h22, 32'h0);
        reg_write(7'h06, 32'd1000, 1'b0);
        read_expect("TICK_DIV locked while armed", 7'h06, 32'd10_000_000);
        reg_write(7'h21, 32'd7, 1'b0);
        read_expect("WDT_TICKS locked while armed", 7'h21, 32'd3);

        reg_write(7'h20, 32'h0, 1'b0);                    // any other value disarms
        read_expect("disarmed by ARM write", 7'h20, 32'hA);
        #20;
        check("LED pins off when disarmed", {16'h0, led}, 32'h0);
        read_expect("LEDS still reads the commanded value", 7'h03, 32'h0000_00FF);
        check("no fault from a commanded disarm", {31'h0, dut.fault_any}, 32'h0);

        reg_write(7'h20, ARM_KEY ^ 32'h1, 1'b0);          // wrong key
        read_expect("wrong key refused", 7'h20, 32'hA);
        reg_write(7'h06, 32'd0, 1'b0);                    // tick off
        reg_write(7'h20, ARM_KEY, 1'b0);
        read_expect("arming refused with tick off", 7'h20, 32'h2);
        reg_write(7'h21, 32'd0, 1'b0);
        read_expect("WDT_TICKS 0 stored as 1", 7'h21, 32'd1);
        reg_write(7'h21, 32'd3, 1'b0);

        // Watchdog: 20 us ticks, limit 3. Only WDT_KICK feeds it.
        reg_write(7'h08, 32'hF, 1'b0);
        reg_write(7'h09, 32'h8, 1'b0);                    // fault IRQ only
        reg_write(7'h06, 32'd2000, 1'b0);
        arm;
        #20;
        check("LED pins on when armed", {16'h0, led}, 32'h0000_80FF);
        for (i = 0; i < 15; i = i + 1) begin
            @(posedge tick);
            #(2000);
            reg_write(7'h24, 32'h0, 1'b0);
        end
        read_expect("armed while kicked", 7'h20, 32'hB);
        check("no fault IRQ while kicked", {31'h0, irq}, 32'h0);
        @(posedge tick);
        #(2000);
        reg_write(7'h24, 32'h0, 1'b0);                    // last kick at T
        t0 = $time;
        while ($time - t0 < 50_000)                       // frames, but no kicks
            reg_read(7'h02, 1'b0, d, ok);
        while ($time - t0 < 52_000) #100;
        read_expect("armed ~2.9 periods after the last kick", 7'h20, 32'hB);
        #(26_000);                                        // past T + 4 periods
        check("FAULT.WDT latched", {28'h0, dut.u_regs.fault_st}, 32'h1);
        check("LED pins: fault on, armed off, outputs off", {16'h0, led}, 32'h0000_4000);
        check("FAULT_N low while latched", {31'h0, fault_n}, 32'h0);
        check("fault IRQ", {31'h0, irq}, 32'h1);
        read_expect("ARM: disarmed, fault", 7'h20, 32'hE);
        read_expect("FAULT_STATUS = WDT", 7'h22, 32'h1);
        read_expect("WDT_KICK reads 0 when disarmed", 7'h24, 32'h0);

        reg_write(7'h20, ARM_KEY, 1'b0);
        read_expect("arming refused while fault latched", 7'h20, 32'hE);
        reg_write(7'h23, 32'h2, 1'b0);                    // wrong bit
        read_expect("FAULT_CLEAR of another bit keeps WDT", 7'h22, 32'h1);
        reg_write(7'h23, 32'h1, 1'b0);
        read_expect("FAULT_CLEAR clears WDT", 7'h22, 32'h0);
        check("fault_any cleared", {31'h0, dut.fault_any}, 32'h0);
        reg_write(7'h08, 32'h8, 1'b0);
        reg_write(7'h09, 32'h0, 1'b0);

        // Slow tick (1 ms) for the remaining faults, so frames in between
        // don't need kicks
        reg_write(7'h06, 32'd100_000, 1'b0);

        // LINK: 4 rejected frames in a row
        arm;
        reg_write(7'h02, 32'h0, 1'b1);
        reg_write(7'h02, 32'h0, 1'b1);
        reg_write(7'h02, 32'h0, 1'b1);
        reg_write(7'h24, 32'h0, 1'b0);                    // a good frame resets the run
        reg_write(7'h02, 32'h0, 1'b1);
        reg_write(7'h02, 32'h0, 1'b1);
        reg_write(7'h02, 32'h0, 1'b1);
        read_expect("armed: never 4 bad frames in a row", 7'h20, 32'hB);
        for (i = 0; i < 4; i = i + 1)
            reg_write(7'h02, 32'h0, 1'b1);
        read_expect("FAULT_STATUS = LINK", 7'h22, 32'h2);
        read_expect("disarmed by LINK", 7'h20, 32'hE);
        reg_write(7'h23, 32'hF, 1'b0);
        read_expect("LINK cleared", 7'h22, 32'h0);

        // E-stop (btnD): latches, can't be cleared while held
        arm;
        reg_write(7'h24, 32'h0, 1'b0);
        btnD = 1'b1;
        wait_inputs;
        check("LED pins after e-stop", {16'h0, led}, 32'h0000_4000);
        read_expect("FAULT_STATUS = ESTOP", 7'h22, 32'h4);
        reg_write(7'h23, 32'hF, 1'b0);
        read_expect("ESTOP not cleared while held", 7'h22, 32'h4);
        reg_write(7'h20, ARM_KEY, 1'b0);
        read_expect("arming refused during e-stop", 7'h20, 32'hE);
        btnD = 1'b0;
        wait_inputs;
        read_expect("ESTOP stays latched after release", 7'h22, 32'h4);
        reg_write(7'h23, 32'h4, 1'b0);
        read_expect("ESTOP cleared after release", 7'h22, 32'h0);

        // ARM_LOST: SW15 down while armed
        arm;
        reg_write(7'h24, 32'h0, 1'b0);
        sw[15] = 1'b0;
        wait_inputs;
        read_expect("FAULT_STATUS = ARM_LOST", 7'h22, 32'h8);
        read_expect("ARM: disarmed, SW15 down, fault", 7'h20, 32'hC);
        reg_write(7'h23, 32'h8, 1'b0);
        reg_write(7'h20, ARM_KEY, 1'b0);
        read_expect("arming refused with SW15 down", 7'h20, 32'h8);
        check("SW15 down while disarmed is no fault", {31'h0, dut.fault_any}, 32'h0);
        sw[15] = 1'b1;
        wait_inputs;
        arm;

        // Snapshot: one consistent sample per tick, even when a tick lands
        // in the middle of a burst read (20 us ticks)
        reg_write(7'h20, 32'h0, 1'b0);
        reg_write(7'h06, 32'd2000, 1'b0);
        @(posedge tick);
        #(1000);
        snap_read(s_a[0], s_a[1], s_a[2], s_a[3]);
        reg_read(7'h07, 1'b0, d, ok);
        check("SNAP_TICK = TICK_COUNT", s_a[0], d);
        check("SNAP_INPUTS = switches at the tick", s_a[2], 32'h0000_8000);
        check("SNAP_SAFETY = ARM bits at the tick", s_a[3], 32'h0000_000A);
        reg_read(7'h0C, 1'b0, d, ok);
        if (d - s_a[1] > 32'd30) begin                  // two reads since the tick
            errors = errors + 1;
            $display("  FAIL SNAP_TIME_US %0d us old", d - s_a[1]);
        end
        checks = checks + 1;
        @(posedge tick);
        t0 = $time;
        #(1000);
        snap_read(s_a[0], s_a[1], s_a[2], s_a[3]);        // between ticks
        while ($time - t0 < 15_000) #100;                 // next tick ~5 us into B
        snap_read(s_b[0], s_b[1], s_b[2], s_b[3]);
        check("a tick fired during burst B", {31'h0, dut.u_regs.tick_count == s_a[0] + 1}, 32'h1);
        for (i = 0; i < 4; i = i + 1)
            check("mid-burst tick: B is the same sample as A", s_b[i], s_a[i]);
        snap_read(s_c[0], s_c[1], s_c[2], s_c[3]);
        check("after the burst: next SNAP_TICK", s_c[0], s_a[0] + 1);
        check("next snapshot 20 us later", s_c[1] - s_a[1], 32'd20);
        reg_write(7'h06, 32'd0, 1'b0);

        // IRQ: CRC error bit (bit 2), enabled after the fact
        reg_write(7'h08, 32'h7, 1'b0);            // clear all
        reg_write(7'h09, 32'h4, 1'b0);            // enable CRC IRQ
        #20;
        check("IRQ low with nothing pending", {31'h0, irq}, 32'h0);
        reg_write(7'h02, 32'h0, 1'b1);            // bad frame
        #20;
        check("IRQ high after CRC error", {31'h0, irq}, 32'h1);
        reg_write(7'h08, 32'h4, 1'b0);            // W1C
        #20;
        check("IRQ low after W1C", {31'h0, irq}, 32'h0);
        reg_write(7'h09, 32'h0, 1'b0);

        // LOOP_TICK at 100 kHz (TICK_DIV = 1000) for 200 us
        reg_write(7'h0D, 32'h1, 1'b0);            // clear counters
        pulses = 0;
        reg_write(7'h06, 32'd1000, 1'b0);
        #(200_000);
        reg_write(7'h06, 32'd0, 1'b0);
        #(2000);
        if (pulses < 19 || pulses > 21) begin
            errors = errors + 1;
            $display("  FAIL LOOP_TICK pulses: %0d (expected ~20)", pulses);
        end
        checks = checks + 1;
        reg_read(7'h07, 1'b0, d, ok);
        check("TICK_COUNT matches pin pulses", d, pulses);

        // Tick pulse width ~1 us
        reg_write(7'h06, 32'd100_000, 1'b0);      // 1 kHz
        @(posedge tick); t0 = $time;
        @(negedge tick); t1 = $time;
        reg_write(7'h06, 32'd0, 1'b0);
        pulse_width = t1 - t0;
        if (pulse_width < 990 || pulse_width > 1010) begin
            errors = errors + 1;
            $display("  FAIL LOOP_TICK width %0d ns", pulse_width);
        end
        checks = checks + 1;

        // Tick IRQ
        reg_write(7'h08, 32'h7, 1'b0);
        reg_write(7'h09, 32'h1, 1'b0);
        reg_write(7'h06, 32'd1000, 1'b0);
        @(posedge tick);
        #20;
        check("IRQ on tick", {31'h0, irq}, 32'h1);
        reg_write(7'h06, 32'd0, 1'b0);
        reg_write(7'h08, 32'h1, 1'b0);
        reg_write(7'h09, 32'h0, 1'b0);

        // Switch change: debounced value appears, status bit 1 sets
        reg_write(7'h08, 32'h7, 1'b0);
        sw = 16'hBEEF;
        btnU = 1'b1;
        #(2_100_000);   // two debounce samples (1 ms each)
        read_expect("SWITCHES", 7'h04, {11'h0, 5'b00001, 16'hBEEF});
        reg_read(7'h08, 1'b0, d, ok);
        check("switch-change status bit", d & 32'h2, 32'h2);

        // 7-segment register override
        reg_write(7'h05, 32'h001F_1234, 1'b0);
        read_expect("SEG7", 7'h05, 32'h001F_1234);

        // MCU-driven reset clears registers
        reg_write(7'h02, 32'h1111_2222, 1'b0);
        rst_n = 1'b0;
        #(500);
        rst_n = 1'b1;
        #(500);
        read_expect("SCRATCH cleared by FPGA_RST_N", 7'h02, 32'h0);
        read_expect("LEDS cleared by FPGA_RST_N", 7'h03, 32'h0);
        reg_write(7'h06, 32'd100_000, 1'b0);
        reg_write(7'h21, 32'd9, 1'b0);
        arm;
        btnD = 1'b1;
        wait_inputs;
        btnD = 1'b0;
        wait_inputs;
        read_expect("ESTOP latched before reset", 7'h22, 32'h4);
        rst_n = 1'b0;
        #(500);
        rst_n = 1'b1;
        #(500);
        read_expect("ARM after FPGA_RST_N: disarmed, tick off", 7'h20, 32'h2);
        read_expect("FAULT_STATUS cleared by FPGA_RST_N", 7'h22, 32'h0);
        read_expect("WDT_TICKS back to 3", 7'h21, 32'd3);
        read_expect("v3 WDOG address now unmapped", 7'h0E, 32'hDEAD_BEEF);

        check("never armed while a fault is latched (clocks)", armed_with_fault, 32'd0);

        checks = checks + 1;
        if (hold_violations != 0) begin
            errors = errors + 1;
            $display("  FAIL MISO hold: %0d changes within 15 ns of SCK rise", hold_violations);
        end

        $display("=== %0d checks, %0d errors ===", checks, errors);
        if (errors == 0) $display("PASS");
        else             $display("FAIL");
        $finish;
    end

    // Watchdog
    initial begin
        #(100_000_000);
        $display("FAIL: timeout");
        $finish;
    end

endmodule
