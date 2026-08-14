`timescale 1ns / 1ps

//================================================================
// radix4_booth_mult16_pipe
//----------------------------------------------------------------
// Pipelined 16 x 16 signed Radix-4 Modified Booth multiplier.
// Fully synchronous, 4-cycle valid_in -> valid_out latency, fully
// pipelined (accepts one new operand pair every cycle).
//
// No '*' operator, no DSP. Same Booth decode as the combinational
// q15_booth_mult, but the 8 partial products are reduced through
// three registered adder-tree stages instead of one big combinational
// sum, so the critical path is broken into 4 shallow pipeline stages:
//
//   Stage 0 (comb) : Booth-decode b, generate 8 partial products
//   Stage 1 (reg)  : latch the 8 partial products
//   Stage 2 (comb->reg) : pairwise-sum 8 -> 4, latch
//   Stage 3 (comb->reg) : pairwise-sum 4 -> 2, latch
//   Stage 4 (comb->reg) : final sum 2 -> 1, latch as `result`
//
// valid_in shifts through the same 4 registers as valid_out, so
// result/valid_out for a given input pair appear exactly 4 clocks
// after that pair was presented (with valid_in=1).
//
// Verilog-2001 synthesizable subset: no initial blocks, no latches
// (always @* fully assigns every case with a default branch),
// sequential logic only in clocked always blocks.
//================================================================

module radix4_booth_mult16_pipe
(
    input clk,
    input rst,

    input valid_in,

    input  signed [15:0] a,
    input  signed [15:0] b,

    output reg signed [31:0] result,
    output reg               valid_out
);

    //------------------------------------------------------------
    // Stage 0 (combinational): Booth decode + 8 partial products
    //------------------------------------------------------------
    wire signed [33:0] a_ext = {{18{a[15]}}, a};
    wire        [16:0] b_op  = {b, 1'b0};

    reg signed [33:0] pp0, pp1, pp2, pp3, pp4, pp5, pp6, pp7;

    reg [2:0]        grp;
    reg signed [2:0] digit;

    always @(*) begin
        // ---- group 0 ----
        grp = b_op[2:0];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp0 = (a_ext <<< 1);
            3'sd1:  pp0 = a_ext;
            3'sd0:  pp0 = 34'sd0;
            -3'sd1: pp0 = -a_ext;
            -3'sd2: pp0 = -(a_ext <<< 1);
            default: pp0 = 34'sd0;
        endcase

        // ---- group 1 ----
        grp = b_op[4:2];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp1 = (a_ext <<< 1) <<< 2;
            3'sd1:  pp1 = a_ext <<< 2;
            3'sd0:  pp1 = 34'sd0;
            -3'sd1: pp1 = (-a_ext) <<< 2;
            -3'sd2: pp1 = (-(a_ext <<< 1)) <<< 2;
            default: pp1 = 34'sd0;
        endcase

        // ---- group 2 ----
        grp = b_op[6:4];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp2 = (a_ext <<< 1) <<< 4;
            3'sd1:  pp2 = a_ext <<< 4;
            3'sd0:  pp2 = 34'sd0;
            -3'sd1: pp2 = (-a_ext) <<< 4;
            -3'sd2: pp2 = (-(a_ext <<< 1)) <<< 4;
            default: pp2 = 34'sd0;
        endcase

        // ---- group 3 ----
        grp = b_op[8:6];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp3 = (a_ext <<< 1) <<< 6;
            3'sd1:  pp3 = a_ext <<< 6;
            3'sd0:  pp3 = 34'sd0;
            -3'sd1: pp3 = (-a_ext) <<< 6;
            -3'sd2: pp3 = (-(a_ext <<< 1)) <<< 6;
            default: pp3 = 34'sd0;
        endcase

        // ---- group 4 ----
        grp = b_op[10:8];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp4 = (a_ext <<< 1) <<< 8;
            3'sd1:  pp4 = a_ext <<< 8;
            3'sd0:  pp4 = 34'sd0;
            -3'sd1: pp4 = (-a_ext) <<< 8;
            -3'sd2: pp4 = (-(a_ext <<< 1)) <<< 8;
            default: pp4 = 34'sd0;
        endcase

        // ---- group 5 ----
        grp = b_op[12:10];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp5 = (a_ext <<< 1) <<< 10;
            3'sd1:  pp5 = a_ext <<< 10;
            3'sd0:  pp5 = 34'sd0;
            -3'sd1: pp5 = (-a_ext) <<< 10;
            -3'sd2: pp5 = (-(a_ext <<< 1)) <<< 10;
            default: pp5 = 34'sd0;
        endcase

        // ---- group 6 ----
        grp = b_op[14:12];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp6 = (a_ext <<< 1) <<< 12;
            3'sd1:  pp6 = a_ext <<< 12;
            3'sd0:  pp6 = 34'sd0;
            -3'sd1: pp6 = (-a_ext) <<< 12;
            -3'sd2: pp6 = (-(a_ext <<< 1)) <<< 12;
            default: pp6 = 34'sd0;
        endcase

        // ---- group 7 (uses b_op[16] = b[15], the sign bit) ----
        grp = b_op[16:14];
        case (grp)
            3'b000: digit = 3'sd0;
            3'b001: digit = 3'sd1;
            3'b010: digit = 3'sd1;
            3'b011: digit = 3'sd2;
            3'b100: digit = -3'sd2;
            3'b101: digit = -3'sd1;
            3'b110: digit = -3'sd1;
            3'b111: digit = 3'sd0;
            default: digit = 3'sd0;
        endcase
        case (digit)
            3'sd2:  pp7 = (a_ext <<< 1) <<< 14;
            3'sd1:  pp7 = a_ext <<< 14;
            3'sd0:  pp7 = 34'sd0;
            -3'sd1: pp7 = (-a_ext) <<< 14;
            -3'sd2: pp7 = (-(a_ext <<< 1)) <<< 14;
            default: pp7 = 34'sd0;
        endcase
    end

    //------------------------------------------------------------
    // Stage 1 registers: latch the 8 raw partial products
    //------------------------------------------------------------
    reg signed [33:0] pp0_r, pp1_r, pp2_r, pp3_r, pp4_r, pp5_r, pp6_r, pp7_r;
    reg                valid_s1;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            pp0_r <= 34'sd0; pp1_r <= 34'sd0; pp2_r <= 34'sd0; pp3_r <= 34'sd0;
            pp4_r <= 34'sd0; pp5_r <= 34'sd0; pp6_r <= 34'sd0; pp7_r <= 34'sd0;
            valid_s1 <= 1'b0;
        end
        else
        begin
            pp0_r <= pp0; pp1_r <= pp1; pp2_r <= pp2; pp3_r <= pp3;
            pp4_r <= pp4; pp5_r <= pp5; pp6_r <= pp6; pp7_r <= pp7;
            valid_s1 <= valid_in;
        end
    end

    //------------------------------------------------------------
    // Stage 2 (comb reduce 8->4, then register)
    //------------------------------------------------------------
    wire signed [34:0] sum0 = pp0_r + pp1_r;
    wire signed [34:0] sum1 = pp2_r + pp3_r;
    wire signed [34:0] sum2 = pp4_r + pp5_r;
    wire signed [34:0] sum3 = pp6_r + pp7_r;

    reg signed [34:0] sum0_r, sum1_r, sum2_r, sum3_r;
    reg                valid_s2;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            sum0_r <= 35'sd0; sum1_r <= 35'sd0; sum2_r <= 35'sd0; sum3_r <= 35'sd0;
            valid_s2 <= 1'b0;
        end
        else
        begin
            sum0_r <= sum0; sum1_r <= sum1; sum2_r <= sum2; sum3_r <= sum3;
            valid_s2 <= valid_s1;
        end
    end

    //------------------------------------------------------------
    // Stage 3 (comb reduce 4->2, then register)
    //------------------------------------------------------------
    wire signed [35:0] tot0 = sum0_r + sum1_r;
    wire signed [35:0] tot1 = sum2_r + sum3_r;

    reg signed [35:0] tot0_r, tot1_r;
    reg                valid_s3;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            tot0_r <= 36'sd0; tot1_r <= 36'sd0;
            valid_s3 <= 1'b0;
        end
        else
        begin
            tot0_r <= tot0; tot1_r <= tot1;
            valid_s3 <= valid_s2;
        end
    end

    //------------------------------------------------------------
    // Stage 4 (comb final sum 2->1, then register as result)
    //------------------------------------------------------------
    wire signed [36:0] final_sum = tot0_r + tot1_r;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            result    <= 32'sd0;
            valid_out <= 1'b0;
        end
        else
        begin
            result    <= final_sum[31:0];
            valid_out <= valid_s3;
        end
    end

endmodule