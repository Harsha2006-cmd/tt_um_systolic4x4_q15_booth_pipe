`timescale 1ns / 1ps

//================================================================
// radix4_booth_mult16_pipe  (SYNTHESIS-FIXED VERSION)
//----------------------------------------------------------------
// Pipelined 16 x 16 signed Radix-4 Modified Booth multiplier.
// Fully synchronous, 4-cycle valid_in -> valid_out latency, fully
// pipelined (accepts one new operand pair every cycle).
//
// CHANGE FROM ORIGINAL: the Booth decode no longer reuses a single
// shared `grp`/`digit` reg across all 8 groups inside one always
// block. Each group now has its own uniquely-named 3-bit "group"
// wire and its own 3-bit signed "digit" wire, computed with plain
// continuous assignments (no case-based ROM-shaped construct that
// Yosys's proc pass can mistake for an addressed memory). This is
// functionally IDENTICAL to the original -- same truth table, same
// partial products, same reduction tree -- only the RTL coding
// style changed.
//
// No '*' operator, no DSP.
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

    // Each group gets its own uniquely-named select/digit signal --
    // no reg is written and re-read multiple times in the same
    // process, so there is nothing for Yosys to (mis)interpret as
    // addressed/procedural memory.
    wire [2:0] grp0 = b_op[2:0];
    wire [2:0] grp1 = b_op[4:2];
    wire [2:0] grp2 = b_op[6:4];
    wire [2:0] grp3 = b_op[8:6];
    wire [2:0] grp4 = b_op[10:8];
    wire [2:0] grp5 = b_op[12:10];
    wire [2:0] grp6 = b_op[14:12];
    wire [2:0] grp7 = b_op[16:14];

    function signed [2:0] booth_digit;
        input [2:0] grp;
        begin
            case (grp)
                3'b000: booth_digit = 3'sd0;
                3'b001: booth_digit = 3'sd1;
                3'b010: booth_digit = 3'sd1;
                3'b011: booth_digit = 3'sd2;
                3'b100: booth_digit = -3'sd2;
                3'b101: booth_digit = -3'sd1;
                3'b110: booth_digit = -3'sd1;
                3'b111: booth_digit = 3'sd0;
                default: booth_digit = 3'sd0;
            endcase
        end
    endfunction

    wire signed [2:0] digit0 = booth_digit(grp0);
    wire signed [2:0] digit1 = booth_digit(grp1);
    wire signed [2:0] digit2 = booth_digit(grp2);
    wire signed [2:0] digit3 = booth_digit(grp3);
    wire signed [2:0] digit4 = booth_digit(grp4);
    wire signed [2:0] digit5 = booth_digit(grp5);
    wire signed [2:0] digit6 = booth_digit(grp6);
    wire signed [2:0] digit7 = booth_digit(grp7);

    function signed [33:0] booth_pp;
        input signed [2:0]  digit;
        input signed [33:0] operand;
        begin
            case (digit)
                3'sd2:   booth_pp = (operand <<< 1);
                3'sd1:   booth_pp = operand;
                3'sd0:   booth_pp = 34'sd0;
                -3'sd1:  booth_pp = -operand;
                -3'sd2:  booth_pp = -(operand <<< 1);
                default: booth_pp = 34'sd0;
            endcase
        end
    endfunction

    wire signed [33:0] pp0 = booth_pp(digit0, a_ext);
    wire signed [33:0] pp1 = booth_pp(digit1, a_ext) <<< 2;
    wire signed [33:0] pp2 = booth_pp(digit2, a_ext) <<< 4;
    wire signed [33:0] pp3 = booth_pp(digit3, a_ext) <<< 6;
    wire signed [33:0] pp4 = booth_pp(digit4, a_ext) <<< 8;
    wire signed [33:0] pp5 = booth_pp(digit5, a_ext) <<< 10;
    wire signed [33:0] pp6 = booth_pp(digit6, a_ext) <<< 12;
    wire signed [33:0] pp7 = booth_pp(digit7, a_ext) <<< 14;

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
