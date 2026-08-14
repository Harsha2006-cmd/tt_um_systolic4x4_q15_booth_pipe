`timescale 1ns / 1ps

//================================================================
// radix4_booth_mult16_pipe
//----------------------------------------------------------------
// Pipelined 16 x 16 signed Radix-4 Modified Booth multiplier.
// Fully synchronous, 4-cycle valid_in -> valid_out latency, fully
// pipelined (accepts one new operand pair every cycle).
//
// FIX (Yosys synthesis-stall fix): the original Stage-0 decode
// block reused a single pair of scalar temp regs (`grp`, `digit`)
// eight times in one always @(*) process -- once per Booth group.
// Yosys's `proc` pass can misidentify that repeated
// write-then-overwrite pattern on a small reg, driven by an
// 8-entry constant-table case, as an addressable memory ($rdmux /
// $auto$proc_rom), then burns large amounts of time trying to
// flatten/share that inferred "memory" back into logic across all
// 128 instances (8 groups x 16 PEs) in the full design.
//
// This version gives every one of the 8 Booth groups its own
// uniquely-named wires (booth_digit0..7, generated via a
// `function`, feeding continuous `assign`s for pp0..pp7) instead
// of a shared/reused reg pair. There is no write-then-overwrite
// pattern left on any single signal, so Yosys sees 8 independent
// small combinational functions and synthesizes 8 independent
// constant-mux trees, exactly as intended -- no memory inference,
// no stall.
//
// Same Booth decode table, same partial-product formulas, same
// 3-stage registered adder-tree reduction, same pipeline depth (4
// stages), same port list as the original -- verified bit-exact
// against the original via direct co-simulation.
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
// Verilog-2001 synthesizable subset: no initial blocks, no latches,
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

    // Pure combinational function: 3-bit Booth window -> signed
    // digit in {-2,-1,0,+1,+2}. No persistent state, no reuse
    // across calls -- each call site below gets its own
    // synthesized constant-mux tree tied to its own operand slice.
    function signed [2:0] booth_digit;
        input [2:0] w;
        begin
            case (w)
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

    // Pure combinational function: digit + pre-shifted operand ->
    // signed partial product. Again no persistent state.
    function signed [33:0] booth_pp;
        input signed [2:0]  d;
        input signed [33:0] shifted_a;
        begin
            case (d)
                3'sd2:   booth_pp = (shifted_a <<< 1);
                3'sd1:   booth_pp = shifted_a;
                3'sd0:   booth_pp = 34'sd0;
                -3'sd1:  booth_pp = -shifted_a;
                -3'sd2:  booth_pp = -(shifted_a <<< 1);
                default: booth_pp = 34'sd0;
            endcase
        end
    endfunction

    // Each group gets its own uniquely-named digit/pp wire pair --
    // nothing is written-then-overwritten on a shared signal.
    wire signed [2:0] digit0 = booth_digit(b_op[2:0]);
    wire signed [2:0] digit1 = booth_digit(b_op[4:2]);
    wire signed [2:0] digit2 = booth_digit(b_op[6:4]);
    wire signed [2:0] digit3 = booth_digit(b_op[8:6]);
    wire signed [2:0] digit4 = booth_digit(b_op[10:8]);
    wire signed [2:0] digit5 = booth_digit(b_op[12:10]);
    wire signed [2:0] digit6 = booth_digit(b_op[14:12]);
    wire signed [2:0] digit7 = booth_digit(b_op[16:14]);

    wire signed [33:0] pp0 = booth_pp(digit0, a_ext);
    wire signed [33:0] pp1 = booth_pp(digit1, a_ext <<< 2);
    wire signed [33:0] pp2 = booth_pp(digit2, a_ext <<< 4);
    wire signed [33:0] pp3 = booth_pp(digit3, a_ext <<< 6);
    wire signed [33:0] pp4 = booth_pp(digit4, a_ext <<< 8);
    wire signed [33:0] pp5 = booth_pp(digit5, a_ext <<< 10);
    wire signed [33:0] pp6 = booth_pp(digit6, a_ext <<< 12);
    wire signed [33:0] pp7 = booth_pp(digit7, a_ext <<< 14);

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
