// tb_replay.v
// Plays the exact MOSI frames recorded from the STM32 driver (frames.hex,
// written by test/host_test) into the RTL, and saves the MISO bytes to
// miso.hex so the driver can parse the FPGA's real responses.

`timescale 1ns/1ps

module tb_replay;

    reg clk = 1'b0;
    always #5 clk = ~clk;

    reg  sck = 1'b0, mosi = 1'b0, cs_n = 1'b1;
    wire miso, irq, tick, fault_n, dp;
    wire [15:0] led;
    wire [6:0] seg;
    wire [3:0] an;

    basys3_top dut (
        .clk(clk), .sw(16'h0), .btnC(1'b0), .btnU(1'b0), .btnL(1'b0),
        .btnR(1'b0), .btnD(1'b0), .led(led), .seg(seg), .dp(dp), .an(an),
        .spi_sck(sck), .spi_mosi(mosi), .spi_miso(miso), .spi_cs_n(cs_n),
        .fpga_irq(irq), .loop_tick(tick), .fpga_rst_n(1'b1), .fault_n(fault_n)
    );

    real half_ns = 47.0;   // 10.6 MHz
    reg  miso_seen = 1'b0;
    always @(miso) miso_seen <= #8 miso;

    integer fin, fout, n, len, i, b, frames;
    reg [7:0] tx, rx;

    initial begin
        fin  = $fopen("frames.hex", "r");
        fout = $fopen("miso.hex", "w");
        if (fin == 0) begin
            $display("FAIL: cannot open frames.hex");
            $finish;
        end
        #(4000);
        frames = 0;
        while (!$feof(fin)) begin
            n = $fscanf(fin, "%h", len);
            if (n == 1) begin
                cs_n = 1'b0;
                #(half_ns);
                for (i = 0; i < len; i = i + 1) begin
                    n = $fscanf(fin, "%h", tx);
                    for (b = 7; b >= 0; b = b - 1) begin
                        mosi = tx[b];
                        #(half_ns);
                        sck = 1'b1;
                        rx[b] = miso_seen;
                        #(half_ns);
                        sck = 1'b0;
                    end
                    $fwrite(fout, "%02X ", rx);
                end
                $fwrite(fout, "\n");
                #(half_ns);
                cs_n = 1'b1;
                #(500);
                frames = frames + 1;
            end
        end
        $fclose(fin);
        $fclose(fout);
        $display("replayed %0d frames", frames);
        $finish;
    end

endmodule
