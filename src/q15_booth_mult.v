`timescale 1ns / 1ps

//======================================================================
// q15_booth_mult
//---------------------------------------------------------------------
// Synthesis-friendly 16 x 16 signed multiplier.
//
// Interface is intentionally identical to the previous q15_booth_mult:
//
//     a      : signed 16-bit
//     b      : signed 16-bit
//     result : signed 32-bit
//
// Function:
//     result = signed(a) * signed(b)
//
// IMPORTANT:
// - Produces the exact signed 32-bit integer product.
// - NO Q15 rescaling is performed.
// - No rounding or saturation.
// - Same mathematical behavior expected by the existing PE/testbench.
// - Written as a single multiplication expression so Yosys can perform
//   its own technology-aware optimization instead of expanding eight
//   large procedural Booth case networks.
//
// This version is intended as a synthesis/area/runtime optimization.
//======================================================================

module q15_booth_mult
(
    input  signed [15:0] a,
    input  signed [15:0] b,
    output signed [31:0] result
);

    // Explicitly extend both operands to 32 bits before multiplication.
    // This makes the signed interpretation unambiguous to synthesis tools.
    wire signed [31:0] a_ext;
    wire signed [31:0] b_ext;

    assign a_ext = {{16{a[15]}}, a};
    assign b_ext = {{16{b[15]}}, b};

    // Exact signed 16 x 16 -> 32 multiplication.
    assign result = a_ext * b_ext;

endmodule
