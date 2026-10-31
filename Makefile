# fpga_link - top-level tasks
#
#   make sim        RTL testbench (Icarus Verilog)          -> must print PASS
#   make replay     STM32 driver -> RTL -> driver round trip -> must print PASS
#   make test       regs-check + sim + replay
#   make regs       regenerate the register map files from regs/regmap.toml
#   make firmware   build firmware/FPGA_Link (arm-none-eabi-gcc), copy to prebuilt/
#   make clean
#
# Hardware steps (Vivado, flashing, serial) are Python/Tcl scripts in scripts/,
# so they work without make; see README.md.

BUILD := build
RTL   := rtl/spi_link.v rtl/regfile.v rtl/hbridge_pwm.v rtl/quad_decoder.v rtl/basys3_top.v
FW    := firmware/FPGA_Link
DRV   := $(FW)/Core/Src/fpga_link.c
DRVH  := $(FW)/Core/Inc/fpga_link.h $(FW)/Core/Inc/fpga_regs.h

# python3 on CI and Linux; on Windows python3 is often the Store stub, so fall back
PYTHON ?= $(shell python3 -c "print('python3')" 2>/dev/null || echo python)

test: regs-check sim replay

regs:
	$(PYTHON) scripts/gen_regs.py

# fails if a generated file no longer matches regs/regmap.toml
regs-check:
	$(PYTHON) scripts/gen_regs.py --check

$(BUILD):
	mkdir -p $(BUILD)

sim: | $(BUILD)
	iverilog -g2012 -Irtl -o $(BUILD)/tb.vvp sim/tb_basys3_top.v $(RTL)
	vvp $(BUILD)/tb.vvp | tee $(BUILD)/sim.log
	@# the testbench always calls $$finish, so judge it by its PASS line
	@grep -q '^PASS' $(BUILD)/sim.log

$(BUILD)/host_test: test/host_test.c $(DRV) $(DRVH) | $(BUILD)
	gcc -std=c11 -Wall -Wextra -Werror -DFPGA_LINK_HOST_TEST -Itest -I$(FW)/Core/Inc \
	    -o $@ test/host_test.c $(DRV)

replay: $(BUILD)/host_test
	cd $(BUILD) && ./host_test record frames.hex
	iverilog -g2012 -Irtl -o $(BUILD)/replay.vvp sim/tb_replay.v $(RTL)
	cd $(BUILD) && vvp replay.vvp
	cd $(BUILD) && ./host_test check miso.hex

firmware:
	$(MAKE) -C $(FW)
	mkdir -p prebuilt
	cp $(FW)/build/FPGA_Link.bin $(FW)/build/FPGA_Link.hex $(FW)/build/FPGA_Link.elf prebuilt/

clean:
	rm -rf $(BUILD)
	$(MAKE) -C $(FW) clean

.PHONY: test regs regs-check sim replay firmware clean
