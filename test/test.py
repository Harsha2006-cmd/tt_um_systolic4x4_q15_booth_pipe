# SPDX-FileCopyrightText: (c) 2026 H S Harsha
# SPDX-License-Identifier: Apache-2.0
#
# Cocotb testbench for:
# tt_um_systolic4x4_q15_booth_pipe
#
# Protocol:
#   ui_in[7:0]  = byte-serial data input
#
#   uio_in[0]   = strobe
#
#   uio_out[1]  = busy
#   uio_out[2]  = done_pulse
#   uio_out[3]  = load_a_phase
#   uio_out[4]  = load_b_phase
#   uio_out[5]  = compute_phase
#   uio_out[6]  = read_phase
#   uio_out[7]  = byte_valid
#
# LOAD_A:
#   32 bytes = 16 signed 16-bit elements
#
# LOAD_B:
#   32 bytes = 16 signed 16-bit elements
#
# COMPUTE:
#   wrapper automatically starts the systolic core
#
# READ:
#   64 bytes = 16 signed 32-bit C elements
#
# Matrix elements are sent/read MSB first, row-major.
#

import random

import cocotb
from cocotb.clock import Clock
from cocotb.triggers import ClockCycles, RisingEdge


# ============================================================
# Reference model
# ============================================================

def to_s16(x):
    """Wrap Python integer to signed 16-bit."""
    x &= 0xFFFF
    return x - 0x10000 if x & 0x8000 else x


def to_s32(x):
    """Wrap Python integer to signed 32-bit."""
    x &= 0xFFFFFFFF
    return x - 0x100000000 if x & 0x80000000 else x


def ref_matmul_4x4(a, b):
    """
    Signed 16x16 -> signed 32-bit matrix multiplication.

    No Q15 rescaling is performed.
    """
    c = [[0] * 4 for _ in range(4)]

    for i in range(4):
        for j in range(4):
            acc = 0

            for k in range(4):
                acc += to_s16(a[i][k]) * to_s16(b[k][j])

            c[i][j] = to_s32(acc)

    return c


# ============================================================
# Status helpers
# ============================================================

def get_status(dut):
    """Return uio_out as an integer."""
    return int(dut.uio_out.value)


def get_busy(dut):
    return (get_status(dut) >> 1) & 1


def get_done(dut):
    return (get_status(dut) >> 2) & 1


def get_load_a(dut):
    return (get_status(dut) >> 3) & 1


def get_load_b(dut):
    return (get_status(dut) >> 4) & 1


def get_compute(dut):
    return (get_status(dut) >> 5) & 1


def get_read(dut):
    return (get_status(dut) >> 6) & 1


def get_byte_valid(dut):
    return (get_status(dut) >> 7) & 1


def status_string(dut):
    return (
        f"status=0x{get_status(dut):02X}, "
        f"busy={get_busy(dut)}, "
        f"done={get_done(dut)}, "
        f"load_a={get_load_a(dut)}, "
        f"load_b={get_load_b(dut)}, "
        f"compute={get_compute(dut)}, "
        f"read={get_read(dut)}, "
        f"byte_valid={get_byte_valid(dut)}"
    )


# ============================================================
# Byte-serial driver
# ============================================================

async def strobe_byte(dut, byte_val):
    """
    Present one byte and generate a one-cycle strobe.
    """

    dut.ui_in.value = byte_val & 0xFF

    # strobe = 1
    dut.uio_in.value = 1

    await RisingEdge(dut.clk)

    # strobe = 0
    dut.uio_in.value = 0

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
    Load a 4x4 matrix.

    16 elements × 2 bytes = 32 bytes.
    """

    for row in mat4x4:
        for val in row:

            hi, lo = elem16_bytes(val)

            await strobe_byte(dut, hi)
            await strobe_byte(dut, lo)


# ============================================================
# Wait for LOAD_A
# ============================================================

async def wait_for_load_a(dut, timeout_cycles=500):
    """
    Wait until the wrapper reports LOAD_A.

    This is important for the second/back-to-back transaction.
    We don't assume that LOAD_A occurs immediately after the
    last read strobe.
    """

    for _ in range(timeout_cycles):

        await RisingEdge(dut.clk)

        if get_load_a(dut) == 1:
            return

    raise TimeoutError(
        "Design did not return to LOAD_A within timeout. "
        + status_string(dut)
    )


# ============================================================
# Wait for done
# ============================================================

async def wait_for_done(dut, timeout_cycles=2000):
    """
    Wait for done_pulse.

    A generous timeout is used because the systolic array is
    pipelined and the exact latency can change with the RTL.
    """

    for _ in range(timeout_cycles):

        await RisingEdge(dut.clk)

        if get_done(dut):
            return

    raise TimeoutError(
        "Core did not assert done_pulse within timeout. "
        + status_string(dut)
    )


# ============================================================
# Read matrix C
# ============================================================

async def read_matrix_c(dut):
    """
    Read 64 output bytes.

    16 elements × 4 bytes = 64 bytes.

    MSB first, row-major.
    """

    bytes_out = []

    # --------------------------------------------------------
    # First byte
    # --------------------------------------------------------
    #
    # After done_pulse the first C byte should become valid.
    #

    await RisingEdge(dut.clk)

    if not get_byte_valid(dut):
        raise AssertionError(
            "byte_valid not set for first C byte. "
            + status_string(dut)
        )

    bytes_out.append(int(dut.uo_out.value) & 0xFF)

    # --------------------------------------------------------
    # Remaining 63 bytes
    # --------------------------------------------------------

    for byte_number in range(1, 64):

        # Generate strobe.
        dut.uio_in.value = 1

        await RisingEdge(dut.clk)

        dut.uio_in.value = 0

        await RisingEdge(dut.clk)

        if not get_byte_valid(dut):
            raise AssertionError(
                f"byte_valid not set while reading C byte "
                f"{byte_number}. "
                + status_string(dut)
            )

        bytes_out.append(int(dut.uo_out.value) & 0xFF)

    assert len(bytes_out) == 64

    # --------------------------------------------------------
    # Convert bytes back to signed 32-bit matrix
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


# ============================================================
# Reset
# ============================================================

async def reset_dut(dut):

    dut.ena.value = 1

    dut.ui_in.value = 0

    dut.uio_in.value = 0

    dut.rst_n.value = 0

    await ClockCycles(dut.clk, 10)

    dut.rst_n.value = 1

    await ClockCycles(dut.clk, 5)


# ============================================================
# Run one complete matrix multiplication
# ============================================================

async def run_one_matmul(dut, a, b, wait_for_ready=True):
    """
    Run:

        LOAD_A
          ↓
        LOAD_B
          ↓
        COMPUTE
          ↓
        DONE
          ↓
        READ

    wait_for_ready=True makes the helper wait until the wrapper
    has returned to LOAD_A before starting a new transaction.
    """

    # --------------------------------------------------------
    # Make sure the wrapper is ready for a new matrix.
    # --------------------------------------------------------

    if wait_for_ready:
        await wait_for_load_a(dut)

    # --------------------------------------------------------
    # Load A
    # --------------------------------------------------------

    if not get_load_a(dut):
        raise AssertionError(
            "Attempting to load A but design is not in LOAD_A. "
            + status_string(dut)
        )

    await load_matrix(dut, a)

    # --------------------------------------------------------
    # Load B
    # --------------------------------------------------------

    if not get_load_b(dut):
        raise AssertionError(
            "After loading A, design did not enter LOAD_B. "
            + status_string(dut)
        )

    await load_matrix(dut, b)

    # --------------------------------------------------------
    # Wait for computation
    # --------------------------------------------------------

    await wait_for_done(dut)

    # --------------------------------------------------------
    # Read C
    # --------------------------------------------------------

    c = await read_matrix_c(dut)

    return c


# ============================================================
# Matrix comparison
# ============================================================

def assert_matrices_equal(actual, expected, label):

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

    """Reset behavior."""

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    status = get_status(dut)

    busy = (status >> 1) & 1

    load_a_phase = (status >> 3) & 1

    assert busy == 0, (
        "Design should not be busy immediately after reset"
    )

    assert load_a_phase == 1, (
        "Design should be in LOAD_A after reset"
    )


# ============================================================
# TEST 2
# ============================================================

@cocotb.test()
async def test_identity_times_b(dut):

    """A = identity, therefore C = B."""

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
        b,
        wait_for_ready=False
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

    """A = all ones."""

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
        b,
        wait_for_ready=False
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

    for row in c:
        assert row == [
            28,
            32,
            36,
            40
        ]


# ============================================================
# TEST 4
# ============================================================

@cocotb.test()
async def test_signed_positive_and_negative(dut):

    """Mixed signed values."""

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
        b,
        wait_for_ready=False
    )

    expected = ref_matmul_4x4(
        a,
        b
    )

    assert_matrices_equal(
        c,
        expected,
        "signed pos/neg"
    )


# ============================================================
# TEST 5
# ============================================================

@cocotb.test()
async def test_zero_matrix(dut):

    """Zero matrix."""

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
        b,
        wait_for_ready=False
    )

    for row in c:
        assert row == [
            0,
            0,
            0,
            0
        ]


# ============================================================
# TEST 6
# ============================================================

@cocotb.test()
async def test_nontrivial_matrix_and_back_to_back(dut):

    """
    Non-trivial matrix multiplication followed by a second
    independent multiplication.

    The second operation explicitly waits for LOAD_A before
    starting. This avoids assuming that the FSM transitions
    to LOAD_A on exactly the same cycle as the final read
    strobe.
    """

    clock = Clock(
        dut.clk,
        20,
        units="ns"
    )

    cocotb.start_soon(clock.start())

    await reset_dut(dut)

    # --------------------------------------------------------
    # First multiplication
    # --------------------------------------------------------

    a = [
        [1, 2, 3, 4],
        [5, 6, 7, 8],
        [9, 10, 11, 12],
        [13, 14, 15, 16],
    ]

    b = [
        [16, 15, 14, 13],
        [12, 11, 10, 9],
        [8, 7, 6, 5],
        [4, 3, 2, 1],
    ]

    expected1 = ref_matmul_4x4(
        a,
        b
    )

    c1 = await run_one_matmul(
        dut,
        a,
        b,
        wait_for_ready=False
    )

    assert_matrices_equal(
        c1,
        expected1,
        "nontrivial run 1"
    )

    # --------------------------------------------------------
    # IMPORTANT:
    #
    # Wait until the wrapper has actually returned to LOAD_A.
    #
    # This is the key change compared with the previous
    # testbench.
    # --------------------------------------------------------

    await wait_for_load_a(dut)

    # --------------------------------------------------------
    # Second multiplication
    # --------------------------------------------------------

    random.seed(1234)

    a2 = [
        [
            random.randint(-1000, 1000)
            for _ in range(4)
        ]
        for _ in range(4)
    ]

    b2 = [
        [
            random.randint(-1000, 1000)
            for _ in range(4)
        ]
        for _ in range(4)
    ]

    expected2 = ref_matmul_4x4(
        a2,
        b2
    )

    c2 = await run_one_matmul(
        dut,
        a2,
        b2,
        wait_for_ready=True
    )

    assert_matrices_equal(
        c2,
        expected2,
        "nontrivial run 2"
    )
