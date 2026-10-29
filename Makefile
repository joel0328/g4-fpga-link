# fpga_link - top-level tasks
#
#   make sim        RTL testbench (Icarus Verilog)          -> must print PASS
#   make replay     STM32 driver -> RTL -> driver round trip -> must print PASS
#   make test       sim + replay
#   make firmware   build firmware/FPGA_Link (arm-none-eabi-gcc), copy to prebuilt/
#   make clean
#
# Hardware steps (Vivado, flashing, serial) are Python/Tcl scripts in scripts/,
# so they work without make; see README.md.

BUILD := build
RTL   := rtl/spi_link.v rtl/regfile.v rtl/basys3_top.v
FW    := firmware/FPGA_Link
DRV   := $(FW)/Core/Src/fpga_link.c
DRVH  := $(FW)/Core/Inc/fpga_link.h $(FW)/Core/Inc/fpga_regs.h

test: sim replay

$(BUILD):
	mkdir -p $(BUILD)

sim: | $(BUILD)
	iverilog -g2012 -o $(BUILD)/tb.vvp sim/tb_basys3_top.v $(RTL)
	vvp $(BUILD)/tb.vvp

$(BUILD)/host_test: test/host_test.c $(DRV) $(DRVH) | $(BUILD)
	gcc -std=c11 -Wall -Wextra -Werror -DFPGA_LINK_HOST_TEST -Itest -I$(FW)/Core/Inc \
	    -o $@ test/host_test.c $(DRV)

replay: $(BUILD)/host_test
	cd $(BUILD) && ./host_test record frames.hex
	iverilog -g2012 -o $(BUILD)/replay.vvp sim/tb_replay.v $(RTL)
	cd $(BUILD) && vvp replay.vvp
	cd $(BUILD) && ./host_test check miso.hex

firmware:
	$(MAKE) -C $(FW)
	mkdir -p prebuilt
	cp $(FW)/build/FPGA_Link.bin $(FW)/build/FPGA_Link.hex $(FW)/build/FPGA_Link.elf prebuilt/

clean:
	rm -rf $(BUILD)
	$(MAKE) -C $(FW) clean

.PHONY: test sim replay firmware clean
