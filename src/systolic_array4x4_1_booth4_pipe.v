`timescale 1ns / 1ps

//================================================================
// systolic_array4x4_1
//----------------------------------------------------------------
// Direct 4x4 extension of systolic_array2x2_1.v. The 2x2 module's
// internal logic (entry skew, valid AND-chain, acc_clear shift
// register, PE grid) is already written generically in terms of
// the localparam N -- only N, the port count, and the port names
// change going from 2x2 -> 4x4. pe_q15 and skew_delay are reused
// completely unmodified.
//
// IMPORTANT (see project write-up): acc_clear must be driven by
// the caller as a single pulse that is COMBINATIONALLY aligned
// with the *same* cycle as the first valid_in pulse of a new
// stream (a rising-edge detector on valid_in, not a registered/
// delayed pulse). See matrix_loader4x4.v for the correct pattern.
//
// ---------------------------------------------------------------
// FIX: the skew_delay instances (A_SKEW / B_SKEW) previously left
// their valid_out port connected-by-name-to-nothing (`.valid_out()`).
// That is legal Verilog for "output intentionally unused", but some
// lint-strict flows escalate an empty named connection to an error
// ("Instance pin connected by name with empty reference"). Fixed by
// connecting each instance's valid_out to a real (still functionally
// unused) wire instead of leaving the port list entry empty. Purely
// a lint fix -- no functional, timing, or interface change, and
// nothing outside this file (including the top module) is touched.
// ---------------------------------------------------------------
//================================================================

module systolic_array4x4_1_booth4_pipe(

    input clk,
    input rst,

    input valid_in,
    input acc_clear,      // pulse for 1 cycle aligned with the FIRST valid_in of a new dot-product stream

    input signed [15:0] a0,
    input signed [15:0] a1,
    input signed [15:0] a2,
    input signed [15:0] a3,

    input signed [15:0] b0,
    input signed [15:0] b1,
    input signed [15:0] b2,
    input signed [15:0] b3,

    output signed [31:0] c00, c01, c02, c03,
    output signed [31:0] c10, c11, c12, c13,
    output signed [31:0] c20, c21, c22, c23,
    output signed [31:0] c30, c31, c32, c33,

    output valid_out
);

    //========================================================
    // Parameters
    //========================================================
    localparam N       = 4;   // matches pe_q15's N=4 / matrix_loader4x4
    localparam PE_LAT  = 6;   // pe_q15's fixed valid_in -> valid_out/acc_valid latency

    //========================================================
    // External row/col buses
    //========================================================
    wire signed [15:0] a_ext [0:N-1];
    wire signed [15:0] b_ext [0:N-1];

    assign a_ext[0] = a0;
    assign a_ext[1] = a1;
    assign a_ext[2] = a2;
    assign a_ext[3] = a3;

    assign b_ext[0] = b0;
    assign b_ext[1] = b1;
    assign b_ext[2] = b2;
    assign b_ext[3] = b3;

    //========================================================
    // Row/column entry skew
    //========================================================
    wire signed [15:0] a_row_in [0:N-1];   // feeds PE[row][0]
    wire signed [15:0] b_col_in [0:N-1];   // feeds PE[0][col]

    // skew_delay's valid_out is not used by this module (validity is
    // tracked separately through the PE grid's own valid AND-chain),
    // but we still give it a real wire per instance instead of an
    // empty named connection.
    wire a_skew_valid_out [0:N-1];
    wire b_skew_valid_out [0:N-1];

    genvar row, col;

    generate
        for(row = 0; row < N; row = row + 1)
        begin : A_ROW_SKEW
            if(row == 0)
                assign a_row_in[row] = a_ext[row];
            else
                skew_delay #(.WIDTH(16), .DELAY(row*PE_LAT)) A_SKEW
                (
                    .clk(clk), .rst(rst),
                    .valid_in(valid_in),
                    .din(a_ext[row]),
                    .dout(a_row_in[row]),
                    .valid_out(a_skew_valid_out[row])
                );
        end
    endgenerate

    generate
        for(col = 0; col < N; col = col + 1)
        begin : B_COL_SKEW
            if(col == 0)
                assign b_col_in[col] = b_ext[col];
            else
                skew_delay #(.WIDTH(16), .DELAY(col*PE_LAT)) B_SKEW
                (
                    .clk(clk), .rst(rst),
                    .valid_in(valid_in),
                    .din(b_ext[col]),
                    .dout(b_col_in[col]),
                    .valid_out(b_skew_valid_out[col])
                );
        end
    endgenerate

    //========================================================
    // acc_clear delay: PE[row][col] sits (row+col)*PE_LAT cycles
    // downstream of the top-level acc_clear pulse.
    //========================================================
    localparam CLR_SR_LEN = 2*(N-1)*PE_LAT;   // = 18 for this 4x4 array

    reg [CLR_SR_LEN-1:0] acc_clear_sr;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
            acc_clear_sr <= {CLR_SR_LEN{1'b0}};
        else
            acc_clear_sr <= {acc_clear_sr[CLR_SR_LEN-2:0], acc_clear};
    end

    //========================================================
    // PE grid
    //========================================================
    wire signed [15:0] a_fwd         [0:N-1][0:N-1];  // PE[row][col].a_out
    wire signed [15:0] b_fwd         [0:N-1][0:N-1];  // PE[row][col].b_out
    wire        [31:0] acc_arr       [0:N-1][0:N-1];
    wire                valid_arr     [0:N-1][0:N-1];
    wire                acc_valid_arr [0:N-1][0:N-1];

    generate
        for(row = 0; row < N; row = row + 1)
        begin : PE_ROW
            for(col = 0; col < N; col = col + 1)
            begin : PE_COL

                wire signed [15:0] a_in_sel = (col == 0) ? a_row_in[row] : a_fwd[row][col-1];
                wire signed [15:0] b_in_sel = (row == 0) ? b_col_in[col] : b_fwd[row-1][col];

                wire valid_in_sel =
                      (row == 0 && col == 0) ? valid_in :
                      (col == 0)             ? valid_arr[row-1][col] :
                      (row == 0)             ? valid_arr[row][col-1] :
                                                (valid_arr[row][col-1] & valid_arr[row-1][col]);

                wire acc_clear_sel = ((row + col) == 0) ? acc_clear :
                                      acc_clear_sr[(row+col)*PE_LAT - 1];

                pe_q15_booth4_pipe #(.N(N)) PE_INST
                (
                    .clk(clk),
                    .rst(rst),

                    .valid_in(valid_in_sel),
                    .acc_clear(acc_clear_sel),

                    .a_in(a_in_sel),
                    .b_in(b_in_sel),

                    .acc_in(32'd0),

                    .a_out(a_fwd[row][col]),
                    .b_out(b_fwd[row][col]),

                    .acc_out(acc_arr[row][col]),

                    .valid_out(valid_arr[row][col]),
                    .acc_valid(acc_valid_arr[row][col])
                );

            end
        end
    endgenerate

    //========================================================
    // Outputs
    //========================================================
    assign c00 = acc_arr[0][0]; assign c01 = acc_arr[0][1]; assign c02 = acc_arr[0][2]; assign c03 = acc_arr[0][3];
    assign c10 = acc_arr[1][0]; assign c11 = acc_arr[1][1]; assign c12 = acc_arr[1][2]; assign c13 = acc_arr[1][3];
    assign c20 = acc_arr[2][0]; assign c21 = acc_arr[2][1]; assign c22 = acc_arr[2][2]; assign c23 = acc_arr[2][3];
    assign c30 = acc_arr[3][0]; assign c31 = acc_arr[3][1]; assign c32 = acc_arr[3][2]; assign c33 = acc_arr[3][3];

    assign valid_out = acc_valid_arr[N-1][N-1];

endmodule
