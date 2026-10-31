// quad_decoder.v
// x4 quadrature decoder for a Hall encoder (JGB37-520: ~7 kHz edges at full
// speed, far below the 100 MHz clock).
//
// Each input goes through a 2-flop synchronizer, then a filter: a new level
// is accepted only after FILTER clocks without a change, so PWM noise
// coupled into the encoder wires can't count. Every accepted edge moves count
// by one: up when A leads B ({A,B} 00 -> 10 -> 11 -> 01), down the other way.
// If A and B are accepted on the same clock the step is ambiguous: count is
// left alone and err pulses for one clock.

module quad_decoder #(
    parameter integer FILTER = 64            // clocks (< 256)
) (
    input  wire        clk,
    input  wire        rst,
    input  wire        a_pin,
    input  wire        b_pin,
    output reg  [31:0] count,                // signed
    output reg         err
);

    reg [1:0] qa_s = 2'b00, qb_s = 2'b00;    // synchronizers (ASYNC_REG in the XDC)
    reg       a_f  = 1'b0,  b_f  = 1'b0;     // filtered levels
    reg [7:0] a_cnt = 8'd0, b_cnt = 8'd0;

    initial begin
        count = 32'h0;
        err   = 1'b0;
    end

    wire a_take = (qa_s[1] != a_f) && (a_cnt == FILTER - 1);
    wire b_take = (qb_s[1] != b_f) && (b_cnt == FILTER - 1);
    wire a_new  = a_take ? qa_s[1] : a_f;
    wire b_new  = b_take ? qb_s[1] : b_f;

    always @(posedge clk) begin
        qa_s <= {qa_s[0], a_pin};
        qb_s <= {qb_s[0], b_pin};
        err  <= 1'b0;

        if (rst) begin
            a_f   <= qa_s[1];
            b_f   <= qb_s[1];
            a_cnt <= 8'd0;
            b_cnt <= 8'd0;
            count <= 32'h0;
        end else begin
            a_cnt <= (qa_s[1] == a_f || a_take) ? 8'd0 : a_cnt + 8'd1;
            b_cnt <= (qb_s[1] == b_f || b_take) ? 8'd0 : b_cnt + 8'd1;
            a_f   <= a_new;
            b_f   <= b_new;

            case ({a_f, b_f, a_new, b_new})
                4'b00_10, 4'b10_11, 4'b11_01, 4'b01_00: count <= count + 32'd1;
                4'b00_01, 4'b01_11, 4'b11_10, 4'b10_00: count <= count - 32'd1;
                4'b00_11, 4'b11_00, 4'b01_10, 4'b10_01: err   <= 1'b1;
                default: ;                       // no change
            endcase
        end
    end

endmodule
