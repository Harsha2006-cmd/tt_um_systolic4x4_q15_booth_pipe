`timescale 1ns / 1ps

module skew_delay
#(
    parameter WIDTH = 16,
    parameter DELAY = 0
)
(
    input clk,
    input rst,

    input valid_in,
    input signed [WIDTH-1:0] din,

    output signed [WIDTH-1:0] dout,
    output valid_out
);

    //----------------------------------------------------------
    // Delay = 0
    //----------------------------------------------------------

generate

if(DELAY == 0)
begin : gen_delay_zero

    assign dout      = din;
    assign valid_out = valid_in;

end

//----------------------------------------------------------
// Delay > 0
//----------------------------------------------------------

else
begin : gen_delay_nonzero

    reg signed [WIDTH-1:0] data_pipe [0:DELAY-1];
    reg                    valid_pipe[0:DELAY-1];

    integer i;

    always @(posedge clk or posedge rst)
    begin

        if(rst)
        begin

            for(i=0;i<DELAY;i=i+1)
            begin

                data_pipe[i]  <= 0;
                valid_pipe[i] <= 0;

            end

        end

        else
        begin

            //----------------------------------
            // Stage-0
            //----------------------------------

            data_pipe[0]  <= din;
            valid_pipe[0] <= valid_in;

            //----------------------------------
            // Remaining stages
            //----------------------------------

            for(i=1;i<DELAY;i=i+1)
            begin

                data_pipe[i]  <= data_pipe[i-1];
                valid_pipe[i] <= valid_pipe[i-1];

            end

        end

    end

    assign dout      = data_pipe[DELAY-1];
    assign valid_out = valid_pipe[DELAY-1];

end

endgenerate

endmodule
