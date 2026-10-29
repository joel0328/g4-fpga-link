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

    reg  [15:0] sw = 16'h0;
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
            read_expect("VERSION", 7'h01, 32'h0000_0001);

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

            // LEDs drive the pins
            reg_write(7'h03, 32'h0000_A5C3, 1'b0);
            #20;
            check("LED pins", {16'h0, led}, 32'h0000_A5C3);
            read_expect("LEDS", 7'h03, 32'h0000_A5C3);

            // Unmapped address
            read_expect("unmapped", 7'h7F, 32'hDEAD_BEEF);

            // Frame counter increases by exactly the number of good frames
            reg_read(7'h0A, 1'b0, frames_before, ok);
            reg_write(7'h02, 32'h1234_5678, 1'b0);
            reg_write(7'h02, 32'h8765_4321, 1'b0);
            reg_read(7'h0A, 1'b0, d, ok);
            // the first read itself, two writes, and this read's own frame
            // completes after its data is latched, so it is not counted yet
            check("FRAMES_OK delta", d - frames_before, 32'd3);
        end
    endtask

    // ------------------------------------------------------------------
    // Main sequence
    // ------------------------------------------------------------------
    reg [31:0] d, prev_cnt, t0, t1;
    reg ok;
    integer pulses;
    integer pulse_width;

    always @(posedge tick) pulses = pulses + 1;

    initial begin
        pulses = 0;
        $display("=== SPI link testbench ===");
        #(4000);   // past power-on reset

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
        #(50_000_000);
        $display("FAIL: timeout");
        $finish;
    end

endmodule
