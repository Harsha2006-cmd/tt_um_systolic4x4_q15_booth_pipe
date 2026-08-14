# Makefile
# See https://docs.cocotb.org/en/stable/quickstart.html for more info

# defaults
SIM ?= icarus
TOPLEVEL_LANG ?= verilog

# Must match info.yaml's source_files list, and paths are relative to this
# Makefile's directory (test/), so we go up one level into src/.
PROJECT_SOURCES = \
	tt_um_systolic4x4_q15_booth_pipe.v \
	systolic_array4x4_booth.v \
	systolic_array4x4_1_booth.v \
	pe_q15_booth.v \
	q15_booth_mult.v \
	matrix_loader4x4.v \
	skew_delay.v

ifneq ($(GATES),yes)

# RTL simulation:
SIM_BUILD		= sim_build/rtl
VERILOG_SOURCES += $(addprefix $(PWD)/../src/,$(PROJECT_SOURCES))

# top level parameters (leave sim as RTL)
COMPILE_ARGS += -DSIM
COMPILE_ARGS += -DFUNCTIONAL
COMPILE_ARGS += -DUSE_POWER_PINS

else

# Gate level simulation:
SIM_BUILD		= sim_build/gl
COMPILE_ARGS    += -DGL_TEST
COMPILE_ARGS    += -DFUNCTIONAL
COMPILE_ARGS    += -DUSE_POWER_PINS
COMPILE_ARGS    += -DSIM
COMPILE_ARGS    += -DUNIT_DELAY=\#1
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/primitives.v
VERILOG_SOURCES += $(PDK_ROOT)/sky130A/libs.ref/sky130_fd_sc_hd/verilog/sky130_fd_sc_hd.v

# this gets copied in by the GDS action workflow
VERILOG_SOURCES += $(PWD)/gate_level_netlist.v

endif

# Use VERILOG_SOURCES + no TOPLEVEL_LANG setting: this makes cocotb use the
# generic Verilog toplevel below
TOPLEVEL = tb

# MODULE is the basename of the Python test file
MODULE = test

# include cocotb's make rules to take care of the simulator setup
include $(shell cocotb-config --makefiles)/Makefile.sim
