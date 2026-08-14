## How it works

A 4x4 signed Q15/INT16 matrix multiplier built as a 16-PE systolic array.
Each processing element accumulates one dot-product term per cycle using a
radix-4 Modified Booth multiplier; the array streams matrix A row-wise and
matrix B column-wise across 4 cycles per matmul (systolic skew handled by
`skew_delay`). The result C = A x B is exact 32-bit signed integer
arithmetic (16x16-bit multiply, 4-term accumulate) -- no fixed-point
rescale-down step is applied.

Because Tiny Tapeout only exposes 24 GPIO pins, this wrapper byte-serializes
the 4x4x16-bit A and B matrices in and the 4x4x32-bit C matrix out, rather
than exposing the matrices in parallel.

## How to test

Drive `ui_in` with successive bytes of matrix A (row-major, MSB-first per
16-bit signed element, 32 bytes total), pulsing `uio_in[0]` (strobe) once
per byte. Repeat for matrix B (another 32 bytes). The design automatically
starts computing once the last B byte is consumed -- `uio_out[1]` (busy)
goes high, and `uio_out[2]` (done_pulse) pulses for one cycle when the
result is ready. Then pulse strobe 64 times to read out matrix C
(row-major, MSB-first per 32-bit signed element), watching `uio_out[7]`
(byte_valid) to confirm each byte on `uo_out` is valid.

See `test/test.py` for a full cocotb-driven example, including an
independent Python reference model.

## External hardware

None. All inputs/outputs are digital, driven through the standard TT
`ui_in` / `uo_out` / `uio` pins.
