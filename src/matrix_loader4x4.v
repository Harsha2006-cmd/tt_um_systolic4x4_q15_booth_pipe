`timescale 1ns / 1ps

//================================================================
// matrix_loader4x4
//
// Captures two parallel 4x4 matrices on "start" and streams them
// into the systolic array over exactly N=4 cycles, one dot-product
// term (k) per cycle:
//
//   a_row0..a_row3 -> A rows streamed over time: A[row][k]
//   b_col0..b_col3 -> B cols streamed over time: B[k][col]
//   valid          -> high during the 4 streaming cycles
//   acc_clear      -> single pulse, COMBINATIONALLY aligned with
//                      the first (k=0) cycle of "valid", i.e. a
//                      rising-edge detector on the internal
//                      streaming state -- NOT a registered/delayed
//                      copy. This is the exact alignment the array
//                      needs; see systolic_array4x4_1.v.
//================================================================

module matrix_loader4x4
#(
    parameter DW = 16
)
(
    input clk,
    input rst,
    input start,

    // Parallel Matrix A (row-major)
    input signed [DW-1:0] a00, a01, a02, a03,
    input signed [DW-1:0] a10, a11, a12, a13,
    input signed [DW-1:0] a20, a21, a22, a23,
    input signed [DW-1:0] a30, a31, a32, a33,

    // Parallel Matrix B (row-major)
    input signed [DW-1:0] b00, b01, b02, b03,
    input signed [DW-1:0] b10, b11, b12, b13,
    input signed [DW-1:0] b20, b21, b22, b23,
    input signed [DW-1:0] b30, b31, b32, b33,

    // Streams to skew network / systolic array
    output reg signed [DW-1:0] a_row0, a_row1, a_row2, a_row3,
    output reg signed [DW-1:0] b_col0, b_col1, b_col2, b_col3,

    output reg valid,
    output      acc_clear
);

    //------------------------------------------------------------
    // Captured matrices
    //------------------------------------------------------------

    reg signed [DW-1:0] a_c [0:3][0:3];
    reg signed [DW-1:0] b_c [0:3][0:3];

    //------------------------------------------------------------
    // Streaming control
    //------------------------------------------------------------

    reg       streaming;
    reg [1:0] k; // k = 0..3 (N = 4 terms per dot product)

    integer i;

    // acc_clear: rising-edge detector on "valid" -> combinationally
    // aligned with the very first (k=0) valid cycle of this stream.
    reg valid_d;
    assign acc_clear = valid && !valid_d;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
            valid_d <= 1'b0;
        else
            valid_d <= valid;
    end

    always @(posedge clk or posedge rst)
    begin

        if(rst)
        begin

            for(i=0;i<4;i=i+1)
            begin
                a_c[0][i] <= 16'd0; a_c[1][i] <= 16'd0;
                a_c[2][i] <= 16'd0; a_c[3][i] <= 16'd0;
                b_c[0][i] <= 16'd0; b_c[1][i] <= 16'd0;
                b_c[2][i] <= 16'd0; b_c[3][i] <= 16'd0;
            end

            a_row0 <= 16'd0; a_row1 <= 16'd0; a_row2 <= 16'd0; a_row3 <= 16'd0;
            b_col0 <= 16'd0; b_col1 <= 16'd0; b_col2 <= 16'd0; b_col3 <= 16'd0;

            valid     <= 1'b0;
            streaming <= 1'b0;
            k         <= 2'd0;

        end

        else
        begin

            //----------------------------------------------
            // Start: latch the parallel matrices, arm streaming
            //----------------------------------------------

            if(start && !streaming)
            begin

                a_c[0][0]<=a00; a_c[0][1]<=a01; a_c[0][2]<=a02; a_c[0][3]<=a03;
                a_c[1][0]<=a10; a_c[1][1]<=a11; a_c[1][2]<=a12; a_c[1][3]<=a13;
                a_c[2][0]<=a20; a_c[2][1]<=a21; a_c[2][2]<=a22; a_c[2][3]<=a23;
                a_c[3][0]<=a30; a_c[3][1]<=a31; a_c[3][2]<=a32; a_c[3][3]<=a33;

                b_c[0][0]<=b00; b_c[0][1]<=b01; b_c[0][2]<=b02; b_c[0][3]<=b03;
                b_c[1][0]<=b10; b_c[1][1]<=b11; b_c[1][2]<=b12; b_c[1][3]<=b13;
                b_c[2][0]<=b20; b_c[2][1]<=b21; b_c[2][2]<=b22; b_c[2][3]<=b23;
                b_c[3][0]<=b30; b_c[3][1]<=b31; b_c[3][2]<=b32; b_c[3][3]<=b33;

                streaming <= 1'b1;
                k         <= 2'd0;

            end

            //----------------------------------------------
            // Stream out one term (k) per cycle:
            //   a_rowR(k) = A[R][k]   (A element moving across row R)
            //   b_colC(k) = B[k][C]   (B element moving down col C)
            //----------------------------------------------

            if(streaming)
            begin

                a_row0 <= a_c[0][k]; a_row1 <= a_c[1][k];
                a_row2 <= a_c[2][k]; a_row3 <= a_c[3][k];

                b_col0 <= b_c[k][0]; b_col1 <= b_c[k][1];
                b_col2 <= b_c[k][2]; b_col3 <= b_c[k][3];

                valid <= 1'b1;

                if(k == 2'd3)
                begin
                    k         <= 2'd0;
                    streaming <= 1'b0;
                end
                else
                begin
                    k <= k + 2'd1;
                end

            end

            else
            begin

                valid <= 1'b0;

            end

        end

    end

endmodule