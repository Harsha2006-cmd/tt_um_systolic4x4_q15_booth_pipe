`timescale 1ns / 1ps

//================================================================
// tt_um_systolic4x4_q15_booth_pipe
//----------------------------------------------------------------
// TinyTapeout top level. Fits the fixed 24-pin GPIO budget
// (ui_in[7:0], uo_out[7:0], uio[7:0]) by byte-serializing the
// 4x4 Q15 matrix I/O, instead of exposing it in parallel.
//
// The compute core (systolic_array4x4_booth4_pipe and everything
// underneath it -- matrix_loader4x4, systolic_array4x4_1_booth4_pipe,
// pe_q15_booth4_pipe, radix4_booth_mult16_pipe, skew_delay) is
// instantiated UNMODIFIED. Only this wrapper is new: a byte-serial
// shift-in / shift-out front end plus a small FSM to sequence it.
//
// Protocol (all on the rising edge of clk, `strobe` = uio_in[0]):
//
//   Phase LOAD_A (bytes 0..63):  ui_in = next byte of matrix A,
//     row-major a00,a01,a02,a03,a10,...,a33, MSB-first per
//     16-bit signed element (2 bytes/element x 16 elements = 32
//     bytes... wait, 16 elements x 2 bytes = 32 bytes for A, then
//     32 bytes for B = 64 bytes total load).
//   Each pulse of `strobe` (held high exactly 1 cycle) consumes
//     one byte from ui_in and advances the internal pointer.
//   After the last B byte is consumed, the wrapper automatically
//     pulses `start` into the core -- no separate control pin
//     needed.
//   Phase COMPUTE: strobes are ignored (uo_out[0]=busy=1) until
//     the core's `done`.
//   Phase READ (bytes 0..63): each `strobe` pulse advances the
//     internal read pointer and the NEXT output byte becomes
//     available on uo_out for the following cycle. Output bytes
//     are row-major c00,c01,...,c33, MSB-first per 32-bit signed
//     element (4 bytes/element x 16 elements = 64 bytes).
//   After the last C byte is read, the wrapper returns to
//     LOAD_A, ready for the next matrix pair.
//
// Status is always visible on uio_out (uio_oe fixed so bit 0 is
// input-only for `strobe`, bits [7:1] are output-only status --
// no bus contention, no direction switching needed):
//   uio_out[1] = busy
//   uio_out[2] = done_pulse (1 cycle, core's done)
//   uio_out[3] = load_a_phase
//   uio_out[4] = load_b_phase
//   uio_out[5] = compute_phase
//   uio_out[6] = read_phase
//   uio_out[7] = byte_valid (uo_out holds a valid C byte this cycle,
//                during READ phase)
//================================================================

module tt_um_systolic4x4_q15_booth_pipe
(
    input  wire [7:0] ui_in,
    output wire [7:0] uo_out,
    input  wire [7:0] uio_in,
    output wire [7:0] uio_out,
    output wire [7:0] uio_oe,
    input  wire        ena,
    input  wire        clk,
    input  wire        rst_n
);

    wire rst = ~rst_n;
    wire strobe = uio_in[0];

    // uio[0] is always input (strobe); uio[7:1] always output (status)
    assign uio_oe = 8'b1111_1110;

    //=========================================================
    // FSM phases
    //=========================================================
    localparam PH_LOAD_A = 3'd0;
    localparam PH_LOAD_B = 3'd1;
    localparam PH_START  = 3'd2;   // 1-cycle pulse into core
    localparam PH_COMPUTE= 3'd3;
    localparam PH_READ   = 3'd4;

    reg [2:0] phase;

    // Per-element byte assembly (16 elements per matrix, 2 bytes each)
    reg [3:0] elem_idx;     // 0..15
    reg       byte_hi;      // 0 = expecting high byte, 1 = expecting low byte
    reg [7:0] hi_byte_r;

    // Read-side byte counters (16 elements, 4 bytes each)
    reg [3:0] r_elem_idx;
    reg [1:0] r_byte_idx;   // 0..3, MSB first

    //=========================================================
    // Storage for the 16 A elements and 16 B elements
    //=========================================================
    reg signed [15:0] a_mem [0:15];
    reg signed [15:0] b_mem [0:15];

    wire signed [31:0] c_mem [0:15];

    //=========================================================
    // Core instantiation (unmodified)
    //=========================================================
    wire core_start;
    wire core_done;
    wire core_busy;
    wire core_valid_out;

    systolic_array4x4_booth4_pipe CORE
    (
        .clk(clk), .rst(rst), .start(core_start),

        .a00(a_mem[0]),  .a01(a_mem[1]),  .a02(a_mem[2]),  .a03(a_mem[3]),
        .a10(a_mem[4]),  .a11(a_mem[5]),  .a12(a_mem[6]),  .a13(a_mem[7]),
        .a20(a_mem[8]),  .a21(a_mem[9]),  .a22(a_mem[10]), .a23(a_mem[11]),
        .a30(a_mem[12]), .a31(a_mem[13]), .a32(a_mem[14]), .a33(a_mem[15]),

        .b00(b_mem[0]),  .b01(b_mem[1]),  .b02(b_mem[2]),  .b03(b_mem[3]),
        .b10(b_mem[4]),  .b11(b_mem[5]),  .b12(b_mem[6]),  .b13(b_mem[7]),
        .b20(b_mem[8]),  .b21(b_mem[9]),  .b22(b_mem[10]), .b23(b_mem[11]),
        .b30(b_mem[12]), .b31(b_mem[13]), .b32(b_mem[14]), .b33(b_mem[15]),

        .c00(c_mem[0]),  .c01(c_mem[1]),  .c02(c_mem[2]),  .c03(c_mem[3]),
        .c10(c_mem[4]),  .c11(c_mem[5]),  .c12(c_mem[6]),  .c13(c_mem[7]),
        .c20(c_mem[8]),  .c21(c_mem[9]),  .c22(c_mem[10]), .c23(c_mem[11]),
        .c30(c_mem[12]), .c31(c_mem[13]), .c32(c_mem[14]), .c33(c_mem[15]),

        .valid_out(core_valid_out),
        .done(core_done),
        .busy(core_busy)
    );

    assign core_start = (phase == PH_START);

    //=========================================================
    // Output byte mux (registered, presented on uo_out)
    //=========================================================
    reg [7:0] out_byte;
    reg        byte_valid_r;
    reg        done_pulse_r;

    always @(posedge clk or posedge rst)
    begin
        if(rst)
        begin
            phase        <= PH_LOAD_A;
            elem_idx     <= 4'd0;
            byte_hi      <= 1'b0;
            hi_byte_r    <= 8'd0;
            r_elem_idx   <= 4'd0;
            r_byte_idx   <= 2'd0;
            out_byte     <= 8'd0;
            byte_valid_r <= 1'b0;
            done_pulse_r <= 1'b0;
        end
        else
        begin
            done_pulse_r <= 1'b0;

            case(phase)

            //--------------------------------------------------
            PH_LOAD_A, PH_LOAD_B:
            begin
                byte_valid_r <= 1'b0;

                if(strobe)
                begin
                    if(!byte_hi)
                    begin
                        // first byte of this element: stash MSB
                        hi_byte_r <= ui_in;
                        byte_hi   <= 1'b1;
                    end
                    else
                    begin
                        // second byte: assemble full element, store, advance
                        if(phase == PH_LOAD_A)
                            a_mem[elem_idx] <= {hi_byte_r, ui_in};
                        else
                            b_mem[elem_idx] <= {hi_byte_r, ui_in};

                        byte_hi <= 1'b0;

                        if(elem_idx == 4'd15)
                        begin
                            elem_idx <= 4'd0;
                            if(phase == PH_LOAD_A)
                                phase <= PH_LOAD_B;
                            else
                                phase <= PH_START;
                        end
                        else
                        begin
                            elem_idx <= elem_idx + 4'd1;
                        end
                    end
                end
            end

            //--------------------------------------------------
            PH_START:
            begin
                // one-cycle pulse (core_start combinationally tied
                // to this phase); move on immediately
                phase <= PH_COMPUTE;
            end

            //--------------------------------------------------
            PH_COMPUTE:
            begin
                byte_valid_r <= 1'b0;

                if(core_done)
                begin
                    done_pulse_r <= 1'b1;
                    phase        <= PH_READ;
                    r_elem_idx   <= 4'd0;
                    r_byte_idx   <= 2'd0;
                    // present first output byte immediately
                    out_byte     <= c_mem[0][31:24];
                    byte_valid_r <= 1'b1;
                end
            end

            //--------------------------------------------------
            PH_READ:
            begin
                if(strobe)
                begin
                    if(r_byte_idx == 2'd3)
                    begin
                        r_byte_idx <= 2'd0;

                        if(r_elem_idx == 4'd15)
                        begin
                            // last byte of last element just strobed out;
                            // return to loading the next matrix pair
                            r_elem_idx   <= 4'd0;
                            phase        <= PH_LOAD_A;
                            byte_valid_r <= 1'b0;
                        end
                        else
                        begin
                            r_elem_idx   <= r_elem_idx + 4'd1;
                            out_byte     <= c_mem[r_elem_idx + 4'd1][31:24];
                            byte_valid_r <= 1'b1;
                        end
                    end
                    else
                    begin
                        r_byte_idx <= r_byte_idx + 2'd1;
                        case(r_byte_idx)
                            2'd0: out_byte <= c_mem[r_elem_idx][23:16];
                            2'd1: out_byte <= c_mem[r_elem_idx][15:8];
                            2'd2: out_byte <= c_mem[r_elem_idx][7:0];
                            default: out_byte <= 8'd0;
                        endcase
                        byte_valid_r <= 1'b1;
                    end
                end
            end

            //--------------------------------------------------
            default: phase <= PH_LOAD_A;

            endcase
        end
    end

    //=========================================================
    // Outputs
    //=========================================================
    assign uo_out = out_byte;

    assign uio_out[0] = 1'b0; // unused (bit is input-only anyway)
    assign uio_out[1] = core_busy;
    assign uio_out[2] = done_pulse_r;
    assign uio_out[3] = (phase == PH_LOAD_A);
    assign uio_out[4] = (phase == PH_LOAD_B);
    assign uio_out[5] = (phase == PH_COMPUTE) || (phase == PH_START);
    assign uio_out[6] = (phase == PH_READ);
    assign uio_out[7] = byte_valid_r;

    // ena is provided by the TT harness but this design has no
    // low-power gating requirement; explicitly reference it so it
    // isn't flagged as unused by lint (no functional effect).
    wire _unused_ena = ena;

endmodule