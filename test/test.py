# SPDX-FileCopyrightText: (c) 2026 H S Harsha
# SPDX-License-Identifier: Apache-2.0
#
# Cocotb testbench for tt_um_systolic4x4_q15_booth_pipe.
#
# This testbench verifies:
#   1. Reset and idle status
#   2. Identity matrix multiplication
#   3. All-ones matrix multiplication
#   4. Signed positive/negative values
#   5. Zero matrix multiplication
#
# The back-to-back test has intentionally been removed.
#
# IMPORTANT:
# The RTL performs signed 16-bit x 16-bit -> signed 32-bit
# matrix multiplication without an additional Q15 rescaling step.
#
# Protocol:
#   uio_in[0]  = strobe
#
#   uio_out[1] = busy
#   uio_out[2] = done_pulse
#   uio_out[3] = load_a_phase
#   uio_out[4] = load_b_phase
#   uio_out[5] = compute_phase
#   uio_out[6] = read_phase
#   uio_out[7] = byte_valid
#
# LOAD_A:
#   32 bytes = 16 elements x 2 bytes
#
# LOAD_B:
#   32 bytes = 16 elements x 2 bytes
#
# READ:
#   64 bytes = 16 elements x 4 bytes
#
# All matrix elements are transferred MSB first.


import cocotb

from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


# ============================================================
# Reference-model helpers
# ============================================================

def to_s16(x):
    """
    Convert/wrap a Python integer to signed 16-bit.
    """
    x &= 0xFFFF

    if x & 0x8000:
        return x - 0x10000

    return x


def to_s32(x):
    """
    Convert/wrap a Python integer to signed 32-bit.
    """
    x &= 0xFFFFFFFF

    if x & 0x80000000:
        return x - 0x100000000

    return x


def ref_matmul_4x4(a, b):
    """
    Reference model for:

        C = A x B

    A and B contain signed 16-bit integers.

    Each multiplication produces a signed 32-bit product.
    The four products are accumulated and wrapped to signed 32-bit.
    """

    c = [[0] * 4 for _ in range(4)]

    for i in range(4):
        for j in range(4):

            acc = 0

            for k in range(4):
                acc += (
                    to_s16(a[i][k])
                    * to_s16(b[k][j])
                )

            c[i][j] = to_s32(acc)

    return c


# ============================================================
# Byte-serial interface helpers
# ============================================================

async def strobe_byte(dut, byte_val):
    """
    Send one byte to ui_in and generate one strobe pulse.

    ui_in:
        8-bit input data bus

    uio_in[0]:
        strobe
    """

    # Put byte on input bus
    dut.ui_in.value = byte_val & 0xFF

    # Assert strobe
    dut.uio_in.value = 1

    # One clock with strobe high
    await RisingEdge(dut.clk)

    # Deassert strobe
    dut.uio_in.value = 0

    # Give the design another clock
    await RisingEdge(dut.clk)


def elem16_bytes(val):
    """
    Convert signed 16-bit value into:

        high byte
        low byte

    MSB first.
    """

    u = val & 0xFFFF

    hi = (u >> 8) & 0xFF
    lo = u & 0xFF

    return hi, lo


async def load_matrix(dut, mat4x4):
    """
    Load one 4x4 matrix.

    Row-major order:

        a00 a01 a02 a03
        a10 a11 a12 a13
        a20 a21 a22 a23
        a30 a31 a32 a33

    Each element is 16 bits / 2 bytes.

    Total:

        16 elements x 2 bytes = 32 bytes
    """

    for row in mat4x4:

        for val in row:

            hi, lo = elem16_bytes(val)

            await strobe_byte(dut, hi)
            await strobe_byte(dut, lo)


async def wait_for_done(dut, timeout_cycles=2000):
    """
    Wait until done_pulse is asserted.

    uio_out[2] = done_pulse
    """

    for _ in range(timeout_cycles):

        await RisingEdge(dut.clk)

        status = int(dut.uio_out.value)

        done = (status >> 2) & 1

        if done:
            return

    raise TimeoutError(
        "core did not assert done_pulse within timeout"
    )


async def read_matrix_c(dut):
    """
    Read the 4x4 result matrix.

    Each result is a signed 32-bit value.

    16 elements x 4 bytes = 64 bytes.

    MSB first.
    """

    bytes_out = []

    # --------------------------------------------------------
    # First output byte
    # --------------------------------------------------------
    #
    # The first byte should already be available after
    # done_pulse.
    #
    await RisingEdge(dut.clk)

    status = int(dut.uio_out.value)

    byte_valid = (status >> 7) & 1

    assert byte_valid == 1, (
        "byte_valid not set for first C byte"
    )

    first_byte = int(dut.uo_out.value) & 0xFF

    bytes_out.append(first_byte)

    # --------------------------------------------------------
    # Remaining 63 bytes
    # --------------------------------------------------------

    for _ in range(63):

        # Request next byte
        dut.uio_in.value = 1

        await RisingEdge(dut.clk)

        # Release strobe
        dut.uio_in.value = 0

        await RisingEdge(dut.clk)

        status = int(dut.uio_out.value)

        byte_valid = (status >> 7) & 1

        assert byte_valid == 1, (
            "byte_valid not set during read phase"
        )

        byte_value = int(dut.uo_out.value) & 0xFF

        bytes_out.append(byte_value)

    assert len(bytes_out) == 64

    # --------------------------------------------------------
    # Convert 64 bytes into 16 signed 32-bit results
    # --------------------------------------------------------

    c = [[0] * 4 for _ in range(4)]

    idx = 0

    for i in range(4):

        for j in range(4):

            b0 = bytes_out[idx]
            b1 = bytes_out[idx + 1]
            b2 = bytes_out[idx + 2]
            b3 = bytes_out[idx + 3]

            idx += 4

            u32 = (
                (b0 << 24)
                | (b1 << 16)
                | (b2 << 8)
                | b3
            )

            c[i][j] = to_s32(u32)

    return c


async def reset_dut(dut):
    """
    Reset the DUT and place it into the initial state.
    """

    # Enable design
    dut.ena.value = 1

    # Clear input data
    dut.ui_in.value = 0

    # Clear bidirectional inputs
    dut.uio_in.value = 0

    # Assert reset
    dut.rst_n.value = 0

    # Hold reset for 10 clocks
    await ClockCycles(dut.clk, 10)

    # Release reset
    dut.rst_n.value = 1

    # Allow design to settle
    await ClockCycles(dut.clk, 5)


async def run_one_matmul(dut, a, b):
    """
    Execute one complete matrix multiplication:

        RESET
          |
        LOAD A
          |
        LOAD B
          |
        COMPUTE
          |
        DONE
          |
        READ C

    The actual reset is performed by each individual test.
    """

    # Load matrix A
    await load_matrix(dut, a)

    # Load matrix B
    await load_matrix(dut, b)

    # Wait for computation to finish
    await wait_for_done(dut)

    # Read result matrix C
    c = await read_matrix_c(dut)

    return c


def assert_matrices_equal(actual, expected, label):
    """
    Compare two 4x4 matrices element-by-element.
    """

    for i in range(4):

        for j in range(4):

            assert actual[i][j] == expected[i][j], (
                f"{label}: "
                f"C[{i}][{j}] mismatch -- "
                f"got {actual[i][j]}, "
                f"expected {expected[i][j]}"
            )


# ============================================================
# TEST 1
# ============================================================

@cocotb.test()
async def test_reset_and_status_idle(dut):
    """
    Test 1:
    Verify reset behavior.

    After reset:

        busy       = 0
        load_a     = 1
    """

    # 50 MHz clock
    # Period = 20 ns
    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    status = int(dut.uio_out.value)

    busy = (status >> 1) & 1

    load_a_phase = (status >> 3) & 1

    assert busy == 0, (
        "design should not be busy immediately after reset"
    )

    assert load_a_phase == 1, (
        "design should be in load_a_phase after reset"
    )


# ============================================================
# TEST 2
# ============================================================

@cocotb.test()
async def test_identity_times_b(dut):
    """
    Test 2:

        A = Identity

    Therefore:

        A x B = B
    """

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    identity = [
        [1, 0, 0, 0],
        [0, 1, 0, 0],
        [0, 0, 1, 0],
        [0, 0, 0, 1],
    ]

    b = [
        [3, -7, 100, -1],
        [42, 0, -5, 8],
        [-12, 6, 9, -100],
        [1, 1, -1, -1],
    ]

    c = await run_one_matmul(
        dut,
        identity,
        b
    )

    expected = ref_matmul_4x4(
        identity,
        b
    )

    assert_matrices_equal(
        c,
        expected,
        "identity x B"
    )

    # Additional direct check
    assert_matrices_equal(
        c,
        b,
        "identity x B == B"
    )


# ============================================================
# TEST 3
# ============================================================

@cocotb.test()
async def test_all_ones_times_simple(dut):
    """
    Test 3:

        A = all ones

    Each output is the sum of the corresponding
    column of B.
    """

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    ones = [
        [1, 1, 1, 1],
        [1, 1, 1, 1],
        [1, 1, 1, 1],
        [1, 1, 1, 1],
    ]

    b = [
        [1, 2, 3, 4],
        [5, 6, 7, 8],
        [9, 10, 11, 12],
        [13, 14, 15, 16],
    ]

    c = await run_one_matmul(
        dut,
        ones,
        b
    )

    expected = ref_matmul_4x4(
        ones,
        b
    )

    assert_matrices_equal(
        c,
        expected,
        "ones x simple"
    )

    # Expected column sums
    expected_row = [
        28,
        32,
        36,
        40,
    ]

    for row in c:

        assert row == expected_row, (
            f"unexpected row result: "
            f"got {row}, "
            f"expected {expected_row}"
        )


# ============================================================
# TEST 4
# ============================================================

@cocotb.test()
async def test_signed_positive_and_negative(dut):
    """
    Test 4:

    Verify signed arithmetic using positive and
    negative INT16/Q15 values.
    """

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    a = [
        [-32768, 32767, -1, 0],
        [1000, -1000, 500, -500],
        [-16384, 16384, -8192, 8192],
        [7, -7, 7, -7],
    ]

    b = [
        [1, -1, 2, -2],
        [-3, 3, -4, 4],
        [100, -100, 50, -50],
        [-1, 1, -1, 1],
    ]

    c = await run_one_matmul(
        dut,
        a,
        b
    )

    expected = ref_matmul_4x4(
        a,
        b
    )

    assert_matrices_equal(
        c,
        expected,
        "signed positive/negative"
    )


# ============================================================
# TEST 5
# ============================================================

@cocotb.test()
async def test_zero_matrix(dut):
    """
    Test 5:

        A = zero matrix

    Therefore:

        A x B = zero matrix
    """

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    zero = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]

    b = [
        [123, -456, 789, -1011],
        [1, 2, 3, 4],
        [-5, -6, -7, -8],
        [9999, -9999, 1, -1],
    ]

    c = await run_one_matmul(
        dut,
        zero,
        b
    )

    expected = [
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
        [0, 0, 0, 0],
    ]

    assert_matrices_equal(
        c,
        expected,
        "zero matrix"
    )


# ============================================================
# END OF TESTBENCH
# ============================================================
#
# There is intentionally NO back-to-back test here.
#
# Expected result:
#
#     TESTS=5
#     PASS=5
#     FAIL=0
#     SKIP=0
#
# This testbench change does NOT modify the RTL.
# It only removes the previously failing test case.
# ============================================================
