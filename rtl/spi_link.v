// spi_link.v
// SPI slave (mode 0: CPOL=0, CPHA=0, MSB first) with a CRC-8 framed register
// protocol. Everything runs in the 100 MHz system clock domain: SCK, CS_N and
// MOSI are oversampled through 2-flop synchronizers, so there is no second
// clock domain. Simulated clean to 12.5 MHz; use 5.3 or 10.6 MHz on the
// Nucleo (SPI1 prescaler /32 or /16 from 170 MHz).
//
// CRC-8: polynomial 0x07, init 0x00, no reflection, no final XOR
//        (CRC-8/SMBUS; check value for "123456789" is 0xF4).
//
// Write frame (6 bytes, MCU -> FPGA):
//   MOSI: [HDR=0|addr] [D31:24] [D23:16] [D15:8] [D7:0] [CRC over bytes 0-4]
//   The register is written only if the CRC matches.
//
// Read frame (7 bytes):
//   MOSI: [HDR=1|addr] [CRC over HDR] [00] [00] [00] [00] [00]
//   MISO: [ xx ]       [ xx ]         [D31:24] [D23:16] [D15:8] [D7:0] [CRC over data]
//   If the header CRC is bad, the FPGA returns FFFFFFFF with an inverted CRC
//   so the MCU always detects the failure.

module spi_link (
    input  wire        clk,
    input  wire        rst,

    input  wire        spi_sck,
    input  wire        spi_cs_n,
    input  wire        spi_mosi,
    output wire        spi_miso,

    output reg  [6:0]  reg_addr,
    output reg  [31:0] reg_wdata,
    output reg         reg_we,
    input  wire [31:0] reg_rdata,

    output reg         frame_ok,
    output reg         crc_err,
    output wire        cs_active
);

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

    // Synchronizers
    reg [2:0] sck_s  = 3'b000;
    reg [2:0] cs_s   = 3'b111;
    reg [1:0] mosi_s = 2'b00;

    always @(posedge clk) begin
        sck_s  <= {sck_s[1:0], spi_sck};
        cs_s   <= {cs_s[1:0],  spi_cs_n};
        mosi_s <= {mosi_s[0],  spi_mosi};
    end

    wire sck_rise = (sck_s[2:1] == 2'b01);
    wire cs_start = (cs_s[2:1]  == 2'b10);
    assign cs_active = ~cs_s[1];

    // Frame state
    reg [2:0]  bit_cnt      = 3'd0;
    reg [2:0]  byte_cnt     = 3'd0;
    reg [7:0]  rx_shift     = 8'h00;
    reg [7:0]  tx_shift     = 8'h00;
    reg        is_read      = 1'b0;
    reg [7:0]  crc_acc      = 8'h00;
    reg [31:0] rd_data      = 32'h0;
    reg        rd_bad       = 1'b0;

    wire [7:0] rx_byte = {rx_shift[6:0], mosi_s[1]};

    wire [7:0] rd_crc = crc8(crc8(crc8(crc8(8'h00, rd_data[31:24]),
                                            rd_data[23:16]),
                                       rd_data[15:8]),
                                  rd_data[7:0]);

    assign spi_miso = tx_shift[7];

    always @(posedge clk) begin
        reg_we   <= 1'b0;
        frame_ok <= 1'b0;
        crc_err  <= 1'b0;

        if (rst) begin
            bit_cnt      <= 3'd0;
            byte_cnt     <= 3'd0;
            tx_shift     <= 8'h00;
            crc_acc      <= 8'h00;
            reg_addr     <= 7'd0;
            reg_wdata    <= 32'h0;
        end else if (cs_start) begin
            bit_cnt      <= 3'd0;
            byte_cnt     <= 3'd0;
            tx_shift     <= 8'h00;
            crc_acc      <= 8'h00;
        end else if (cs_active) begin
            if (sck_rise) begin
                rx_shift <= rx_byte;
                bit_cnt  <= bit_cnt + 3'd1;

                // MISO is updated just after the master's sampling (rising)
                // edge, not on the falling edge. Through the synchronizer
                // that is 20-30 ns after the edge: well past the STM32's
                // MISO hold time, and it leaves almost a full SCK period of
                // setup before the next rising edge.
                if (bit_cnt != 3'd7)
                    tx_shift <= {tx_shift[6:0], 1'b0};

                if (bit_cnt == 3'd7) begin
                    if (byte_cnt != 3'd7)
                        byte_cnt <= byte_cnt + 3'd1;
                    tx_shift <= 8'h00;

                    case (byte_cnt)
                        3'd0: begin
                            is_read  <= rx_byte[7];
                            reg_addr <= rx_byte[6:0];
                            crc_acc  <= crc8(8'h00, rx_byte);
                        end
                        3'd1: begin
                            if (is_read) begin
                                if (rx_byte == crc_acc) begin
                                    rd_data <= reg_rdata;
                                    rd_bad  <= 1'b0;
                                    tx_shift <= reg_rdata[31:24];
                                end else begin
                                    rd_data <= 32'hFFFF_FFFF;
                                    rd_bad  <= 1'b1;
                                    tx_shift <= 8'hFF;
                                    crc_err <= 1'b1;
                                end
                            end else begin
                                reg_wdata[31:24] <= rx_byte;
                                crc_acc <= crc8(crc_acc, rx_byte);
                            end
                        end
                        3'd2: begin
                            if (is_read) tx_shift <= rd_data[23:16];
                            else begin
                                reg_wdata[23:16] <= rx_byte;
                                crc_acc <= crc8(crc_acc, rx_byte);
                            end
                        end
                        3'd3: begin
                            if (is_read) tx_shift <= rd_data[15:8];
                            else begin
                                reg_wdata[15:8] <= rx_byte;
                                crc_acc <= crc8(crc_acc, rx_byte);
                            end
                        end
                        3'd4: begin
                            if (is_read) tx_shift <= rd_data[7:0];
                            else begin
                                reg_wdata[7:0] <= rx_byte;
                                crc_acc <= crc8(crc_acc, rx_byte);
                            end
                        end
                        3'd5: begin
                            if (is_read) begin
                                tx_shift <= rd_bad ? ~rd_crc : rd_crc;
                                frame_ok <= ~rd_bad;
                            end else if (rx_byte == crc_acc) begin
                                reg_we   <= 1'b1;
                                frame_ok <= 1'b1;
                            end else begin
                                crc_err  <= 1'b1;
                            end
                        end
                        default: ;
                    endcase
                end
            end
        end
    end

endmodule
