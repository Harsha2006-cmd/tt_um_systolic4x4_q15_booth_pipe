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
// ---------------------------------------------------------------
// FIX #1 (synthesis stall at Yosys `opt` / proc_rom, phase 5877):
// The previous stage-0 implementation used ONE shared `grp`/`digit`
// reg pair, written 8 times (once per Booth group) inside a single
// `always @(*)` block. Yosys's `proc`/`opt_share`/`opt_muxtree`
// passes see a scalar written repeatedly across many branches in
// one process and start modeling it as a shared mux/ROM structure
// instead of 8 independent combinational cones -- this is what hangs
// `opt` when the block is instantiated 16x (once per PE) across the
// 4x4 array.
//
// The fix moves Booth decode + partial-product generation into a
// single reusable function (`booth_pp`) and calls it via 8 SEPARATE
// continuous assignments (`assign pp0 = booth_pp(...); ...`). Each
// pp_i is now its own independent combinational cone with no shared
// mutable variable across groups, so there is nothing for Yosys to
// (mis)identify as a shared mux tree. This is exactly the pattern
// already proven in this project's fix for the non-pipelined
// q15_booth_mult.
//
// ---------------------------------------------------------------
// FIX #2 (stall further downstream, at $flatten...MULT.$procdff
// during a later opt/constprop pass):
// Even after fix #1, this module is instantiated 16x (once per PE
// in the 4x4 array). When Yosys flattens the design ahead of that
// later optimization pass, it loses the fact that all 16 copies are
// identical -- it re-runs full bit-level constant propagation /
// opt independently on each of the 16 flattened copies instead of
// solving it once. That per-instance blow-up is what stalls the
// flow deep in the log at
// `$flatten\CORE.\ARRAY.\PE_ROW[3].PE_COL[3].PE_INST.\MULT`.
//
// Adding `(* keep_hierarchy = "yes" *)` to this module tells Yosys
// to preserve it as a hierarchy boundary during this pass, so it is
// optimized once and reused (instantiated) 16 times rather than
// flattened + independently re-optimized 16 times. This is a
// synthesis-attribute-only change: it does not alter the module's
// port list, functional behavior, or 4-cycle latency, and downstream
// full-chip hardening flows (e.g. OpenLane's own `synth` step) that
// need a fully flattened netlist will still flatten it normally at
// that later stage regardless of this attribute.
//
// Interface (clk, rst, valid_in, a, b, result, valid_out), the
// 4-cycle latency, and Stages 1-4 (registers + adder tree) are
// completely unchanged -- this file remains a drop-in replacement,
// and nothing upstream (pe_q15_booth4_pipe, systolic_array4x4_1_
// booth4_pipe, systolic_array4x4_booth4_pipe, or the TinyTapeout
// top module) needs to change.
// ---------------------------------------------------------------
//
// Verilog-2001 synthesizable subset: no initial blocks, no latches
// (always @* / functions fully assign every case with a default
// branch), sequential logic only in clocked always blocks.
//================================================================

(* keep_hierarchy = "yes" *)
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

    //------------------------------------------------------------
    // Shared Booth-decode/partial-product function.
    // Pure combinational, no persistent state across calls --
    // each call site below drives its own independent wire, so
    // there is no cross-group sharing for Yosys to trip over.
    //------------------------------------------------------------
    function signed [33:0] booth_pp;
        input signed [33:0] a_val;
        input        [2:0]  win;
        input        [4:0]  shift;

        reg signed [33:0] digit_val;
        begin
            case (win)
                3'b000: digit_val = 34'sd0;
                3'b001: digit_val = a_val;
                3'b010: digit_val = a_val;
                3'b011: digit_val = (a_val <<< 1);
                3'b100: digit_val = -(a_val <<< 1);
                3'b101: digit_val = -a_val;
                3'b110: digit_val = -a_val;
                3'b111: digit_val = 34'sd0;
                default: digit_val = 34'sd0;
            endcase
            booth_pp = digit_val <<< shift;
        end
    endfunction

    //------------------------------------------------------------
    // 8 independent continuous assignments -- one per Booth group.
    // No always block, no shared reg, no proc_rom trap.
    //------------------------------------------------------------
    wire signed [33:0] pp0 = booth_pp(a_ext, b_op[2:0],   5'd0);
    wire signed [33:0] pp1 = booth_pp(a_ext, b_op[4:2],   5'd2);
    wire signed [33:0] pp2 = booth_pp(a_ext, b_op[6:4],   5'd4);
    wire signed [33:0] pp3 = booth_pp(a_ext, b_op[8:6],   5'd6);
    wire signed [33:0] pp4 = booth_pp(a_ext, b_op[10:8],  5'd8);
    wire signed [33:0] pp5 = booth_pp(a_ext, b_op[12:10], 5'd10);
    wire signed [33:0] pp6 = booth_pp(a_ext, b_op[14:12], 5'd12);
    wire signed [33:0] pp7 = booth_pp(a_ext, b_op[16:14], 5'd14); // uses b_op[16] = b[15], the sign bit

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
