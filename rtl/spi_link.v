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
//
// Burst frames (map version 2) use address 0x7F in the header as an escape,
// then name the real start address and a word count N (1..16, sent as N-1).
// Addresses increment from the start address and wrap at 0x7F.
//
// Burst write (5 + 4N bytes):
//   MOSI: [0x7F] [start] [N-1] [CRC over bytes 0-2] [N x 4 data bytes] [CRC over data]
//   The words are buffered and written only if both CRCs match; otherwise
//   nothing is written and CRC_ERRS counts the frame once.
//
// Burst read (5 + 4N bytes):
//   MOSI: [0xFF] [start] [N-1] [CRC over bytes 0-2] [00 ...]
//   MISO: [ xx ] [ xx  ] [xx ] [ xx ]               [N x 4 data bytes] [CRC over data]
//   A bad header CRC or N > 16 returns all-FF data with an inverted CRC.

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

    localparam [6:0] BURST_ADDR = 7'h7F;

    // Burst phases (after the 4 header bytes)
    localparam [1:0] PH_HDR  = 2'd0,
                     PH_DATA = 2'd1,
                     PH_CRC  = 2'd2,   // burst write: waiting for the data CRC
                     PH_DONE = 2'd3;

    // Frame state
    reg [2:0]  bit_cnt      = 3'd0;
    reg [2:0]  byte_cnt     = 3'd0;
    reg [7:0]  rx_shift     = 8'h00;
    reg [7:0]  tx_shift     = 8'h00;
    reg        is_read      = 1'b0;
    reg [7:0]  crc_acc      = 8'h00;
    reg [31:0] rd_data      = 32'h0;
    reg        rd_bad       = 1'b0;   // v1 read: bad header; burst: frame rejected

    // Burst state
    reg        burst        = 1'b0;
    reg [1:0]  b_phase      = PH_HDR;
    reg [6:0]  b_start      = 7'd0;
    reg [3:0]  b_last       = 4'd0;   // N-1
    reg        b_cnt_ok     = 1'b0;   // N-1 <= 15
    reg [1:0]  b_byte       = 2'd0;   // byte within the current word
    reg [3:0]  b_word       = 4'd0;   // current word
    reg [7:0]  tx_crc       = 8'h00;  // CRC of the data bytes sent so far
    reg [23:0] w_hold       = 24'h0;  // first 3 bytes of the word being received
    reg [31:0] wbuf [0:15];           // burst write buffer
    reg        apply        = 1'b0;   // writing wbuf into the register file
    reg [3:0]  apply_idx    = 4'd0;

    wire [7:0] rx_byte = {rx_shift[6:0], mosi_s[1]};

    wire [7:0] rd_crc = crc8(crc8(crc8(crc8(8'h00, rd_data[31:24]),
                                            rd_data[23:16]),
                                       rd_data[15:8]),
                                  rd_data[7:0]);

    // Next byte of the current burst read word (after byte b_byte)
    wire [7:0] rd_next = (b_byte == 2'd0) ? rd_data[23:16] :
                         (b_byte == 2'd1) ? rd_data[15:8]  : rd_data[7:0];

    // Next burst read word (all ones once the frame was rejected)
    wire [31:0] word_next = rd_bad ? 32'hFFFF_FFFF : reg_rdata;

    // Burst header check, evaluated as its CRC byte (byte 3) completes
    wire hdr_ok = (rx_byte == crc_acc) && b_cnt_ok;

    assign spi_miso = tx_shift[7];

    always @(posedge clk) begin
        reg_we   <= 1'b0;
        frame_ok <= 1'b0;
        crc_err  <= 1'b0;

        // Burst write: one buffered word per clock once the frame checked out.
        // This takes at most 16 clocks; the next frame can't reach a byte
        // boundary (where the logic below sets reg_addr) that soon.
        if (apply) begin
            reg_addr  <= b_start + {3'd0, apply_idx};
            reg_wdata <= wbuf[apply_idx];
            reg_we    <= 1'b1;
            apply_idx <= apply_idx + 4'd1;
            if (apply_idx == b_last)
                apply <= 1'b0;
        end

        if (rst) begin
            bit_cnt      <= 3'd0;
            byte_cnt     <= 3'd0;
            tx_shift     <= 8'h00;
            crc_acc      <= 8'h00;
            reg_addr     <= 7'd0;
            reg_wdata    <= 32'h0;
            reg_we       <= 1'b0;
            burst        <= 1'b0;
            b_phase      <= PH_HDR;
            apply        <= 1'b0;
        end else if (cs_start) begin
            bit_cnt      <= 3'd0;
            byte_cnt     <= 3'd0;
            tx_shift     <= 8'h00;
            crc_acc      <= 8'h00;
            burst        <= 1'b0;
            b_phase      <= PH_HDR;
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

                    if (byte_cnt == 3'd0) begin
                        is_read  <= rx_byte[7];
                        reg_addr <= rx_byte[6:0];
                        crc_acc  <= crc8(8'h00, rx_byte);
                        burst    <= (rx_byte[6:0] == BURST_ADDR);
                    end else if (burst) begin
                        if (b_phase == PH_HDR) begin
                            case (byte_cnt)
                                3'd1: begin
                                    b_start  <= rx_byte[6:0];
                                    reg_addr <= rx_byte[6:0];
                                    crc_acc  <= crc8(crc_acc, rx_byte);
                                end
                                3'd2: begin
                                    b_last   <= rx_byte[3:0];
                                    b_cnt_ok <= (rx_byte[7:4] == 4'd0);
                                    crc_acc  <= crc8(crc_acc, rx_byte);
                                end
                                default: begin   // byte 3: header CRC
                                    b_phase <= PH_DATA;
                                    b_byte  <= 2'd0;
                                    b_word  <= 4'd0;
                                    crc_acc <= 8'h00;
                                    rd_bad  <= ~hdr_ok;
                                    if (!hdr_ok)
                                        crc_err <= 1'b1;
                                    if (is_read) begin
                                        rd_data  <= hdr_ok ? reg_rdata : 32'hFFFF_FFFF;
                                        tx_shift <= hdr_ok ? reg_rdata[31:24] : 8'hFF;
                                        tx_crc   <= crc8(8'h00, hdr_ok ? reg_rdata[31:24] : 8'hFF);
                                        reg_addr <= reg_addr + 7'd1;
                                    end
                                end
                            endcase
                        end else if (b_phase == PH_DATA) begin
                            b_byte <= b_byte + 2'd1;
                            if (is_read) begin
                                if (b_byte != 2'd3) begin
                                    tx_shift <= rd_next;
                                    tx_crc   <= crc8(tx_crc, rd_next);
                                end else if (b_word != b_last) begin
                                    b_word   <= b_word + 4'd1;
                                    rd_data  <= word_next;
                                    tx_shift <= word_next[31:24];
                                    tx_crc   <= crc8(tx_crc, word_next[31:24]);
                                    reg_addr <= reg_addr + 7'd1;
                                end else begin
                                    tx_shift <= rd_bad ? ~tx_crc : tx_crc;
                                    frame_ok <= ~rd_bad;
                                    b_phase  <= PH_DONE;
                                end
                            end else begin
                                crc_acc <= crc8(crc_acc, rx_byte);
                                if (b_byte != 2'd3) begin
                                    w_hold <= {w_hold[15:0], rx_byte};
                                end else begin
                                    wbuf[b_word] <= {w_hold, rx_byte};
                                    if (b_word == b_last)
                                        b_phase <= PH_CRC;
                                    else
                                        b_word <= b_word + 4'd1;
                                end
                            end
                        end else if (b_phase == PH_CRC) begin
                            // A rejected header was already counted once
                            b_phase <= PH_DONE;
                            if (!rd_bad) begin
                                if (rx_byte == crc_acc) begin
                                    apply     <= 1'b1;
                                    apply_idx <= 4'd0;
                                    frame_ok  <= 1'b1;
                                end else begin
                                    crc_err   <= 1'b1;
                                end
                            end
                        end
                        // PH_DONE: extra bytes are ignored
                    end else case (byte_cnt)
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
