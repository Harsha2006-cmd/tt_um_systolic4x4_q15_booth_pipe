`timescale 1ns / 1ps

//================================================================
// pe_q15_booth4_pipe
//----------------------------------------------------------------
// Same accumulate-with-clear interface/semantics as pe_q15_booth,
// but built around the fully-pipelined radix4_booth_mult16_pipe
// (4-cycle latency) instead of a combinational multiplier.
//
// Because the multiplier itself now takes 4 cycles to produce
// `product`, a_in/b_in/acc_clear are shifted through a matching
// 4-stage side-channel delay so they land aligned with `product`
// and `product_valid` on the same cycle. Accumulate and output
// then each add one more register stage, exactly as in the
// combinational-Booth PE, giving a total valid_in -> acc_valid
// latency of 4 (mult) + 1 (accumulate) + 1 (output) = 6 cycles.
//================================================================

module pe_q15_booth4_pipe
#(
    parameter N = 2   // number of terms accumulated per dot-product (matrix dimension)
)
(
    input clk,
    input rst,

    input valid_in,
    input acc_clear,      // pulses (aligned with the first valid_in of a new sum) to start a fresh accumulation

    input  signed [15:0] a_in,
    input  signed [15:0] b_in,

    input  signed [31:0] acc_in,   // kept for interface compatibility; not used internally

    output reg signed [15:0] a_out,
    output reg signed [15:0] b_out,

    output reg signed [31:0] acc_out,

    output reg valid_out,   // still pulses every valid cycle, needed for a_out/b_out forwarding
    output reg acc_valid    // pulses exactly once, on the cycle the Nth (final) term lands in acc_out
);

    localparam MULT_LAT = 4;

    //=========================================================
    // Pipelined Booth multiplier (4-cycle latency)
    //=========================================================

    wire signed [31:0] product;
    wire                product_valid;

    radix4_booth_mult16_pipe MULT
    (
        .clk(clk),
        .rst(rst),
        .valid_in(valid_in),
        .a(a_in),
        .b(b_in),
        .result(product),
        .valid_out(product_valid)
    );

    //=========================================================
    // Side-channel delay: a_in / b_in / acc_clear shifted through
    // MULT_LAT stages so they land aligned with product/product_valid
    //=========================================================

    reg signed [15:0] a_dly [0:MULT_LAT-1];
    reg signed [15:0] b_dly [0:MULT_LAT-1];
    reg                acc_clear_dly [0:MULT_LAT-1];

    integer i;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            for(i = 0; i < MULT_LAT; i = i + 1)
            begin
                a_dly[i]         <= 16'd0;
                b_dly[i]         <= 16'd0;
                acc_clear_dly[i] <= 1'b0;
            end
        end
        else
        begin
            a_dly[0]         <= a_in;
            b_dly[0]         <= b_in;
            acc_clear_dly[0] <= acc_clear;

            for(i = 1; i < MULT_LAT; i = i + 1)
            begin
                a_dly[i]         <= a_dly[i-1];
                b_dly[i]         <= b_dly[i-1];
                acc_clear_dly[i] <= acc_clear_dly[i-1];
            end
        end
    end

    wire signed [15:0] a_aligned         = a_dly[MULT_LAT-1];
    wire signed [15:0] b_aligned         = b_dly[MULT_LAT-1];
    wire                acc_clear_aligned = acc_clear_dly[MULT_LAT-1];

    //=========================================================
    // Accumulate stage: persistent per-PE accumulator + term
    // counter, updated the cycle `product`/`product_valid` land
    //=========================================================

    reg signed [31:0] local_acc;
    reg [31:0]         term_cnt;

    reg signed [31:0] acc_sum_r;
    reg                acc_valid_stage;

    reg signed [15:0] a_r2;
    reg signed [15:0] b_r2;

    reg valid_stage2;

    wire signed [31:0] new_acc      = acc_clear_aligned ? product : (local_acc + product);
    wire        [31:0] new_term_cnt = acc_clear_aligned ? 32'd1   : (term_cnt + 32'd1);
    wire                term_done   = product_valid && (new_term_cnt == N);

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            local_acc <= 32'd0;
            term_cnt  <= 32'd0;

            acc_sum_r       <= 32'd0;
            acc_valid_stage <= 1'b0;

            a_r2 <= 16'd0;
            b_r2 <= 16'd0;

            valid_stage2 <= 1'b0;
        end
        else
        begin
            if(product_valid)
            begin
                local_acc <= new_acc;
                term_cnt  <= new_term_cnt;
            end

            acc_sum_r       <= product_valid ? new_acc : local_acc;
            acc_valid_stage <= term_done;

            a_r2 <= a_aligned;
            b_r2 <= b_aligned;

            valid_stage2 <= product_valid;
        end
    end

    //=========================================================
    // Output stage
    //=========================================================

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            acc_out   <= 32'd0;

            a_out     <= 16'd0;
            b_out     <= 16'd0;

            valid_out <= 1'b0;
            acc_valid <= 1'b0;
        end
        else
        begin
            acc_out <= acc_sum_r;

            a_out <= a_r2;
            b_out <= b_r2;

            valid_out <= valid_stage2;
            acc_valid <= acc_valid_stage;
        end
    end

endmodule