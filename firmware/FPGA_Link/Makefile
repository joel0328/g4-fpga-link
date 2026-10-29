# Command-line build for FPGA_Link (STM32G431KB, Nucleo-G431KB).
# STM32CubeIDE ignores this file and uses its own managed build.
# Prebuilt output from this Makefile is in ../../prebuilt/.
#
#   make            -> build/FPGA_Link.elf, .bin, .hex
#   make clean
#   make flash      -> needs STM32CubeProgrammer CLI on PATH
#
# Toolchain: arm-none-eabi-gcc (GNU Arm Embedded / xPack / the one bundled
# with CubeIDE). Override with  make PREFIX=/path/to/arm-none-eabi-

TARGET  := FPGA_Link
BUILD   := build
PREFIX  ?= arm-none-eabi-
CC      := $(PREFIX)gcc
AS      := $(PREFIX)gcc -x assembler-with-cpp
OBJCOPY := $(PREFIX)objcopy
SIZE    := $(PREFIX)size

MCU     := -mcpu=cortex-m4 -mthumb -mfpu=fpv4-sp-d16 -mfloat-abi=hard
DEFS    := -DUSE_HAL_DRIVER -DSTM32G431xx
INCS    := -ICore/Inc \
           -IDrivers/STM32G4xx_HAL_Driver/Inc \
           -IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
           -IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
           -IDrivers/CMSIS/Include
OPT     ?= -Og -g3

APP_SRC := $(wildcard Core/Src/*.c)
HAL_SRC := $(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c)
ASM_SRC := Core/Startup/startup_stm32g431kbtx.s

CFLAGS  := $(MCU) $(DEFS) $(INCS) $(OPT) -std=gnu11 -ffunction-sections -fdata-sections \
           -Wall -fstack-usage -MMD -MP -ffile-prefix-map=$(CURDIR)=.
# Extra warnings for our own code, not ST's
APP_WARN := -Wextra -Werror
LDFLAGS := $(MCU) -T STM32G431KBTX_FLASH.ld --specs=nano.specs --specs=nosys.specs \
           -Wl,-Map=$(BUILD)/$(TARGET).map -Wl,--gc-sections -Wl,--print-memory-usage \
           -Wl,--start-group -lc -lm -Wl,--end-group

APP_OBJ := $(APP_SRC:%.c=$(BUILD)/%.o)
HAL_OBJ := $(HAL_SRC:%.c=$(BUILD)/%.o)
ASM_OBJ := $(ASM_SRC:%.s=$(BUILD)/%.o)

all: $(BUILD)/$(TARGET).elf $(BUILD)/$(TARGET).bin $(BUILD)/$(TARGET).hex

$(BUILD)/Core/Src/%.o: Core/Src/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) $(APP_WARN) -c $< -o $@

$(BUILD)/Drivers/%.o: Drivers/%.c
	@mkdir -p $(dir $@)
	$(CC) $(CFLAGS) -c $< -o $@

$(BUILD)/%.o: %.s
	@mkdir -p $(dir $@)
	$(AS) $(MCU) -c $< -o $@

$(BUILD)/$(TARGET).elf: $(APP_OBJ) $(HAL_OBJ) $(ASM_OBJ)
	$(CC) $^ $(LDFLAGS) -o $@
	$(SIZE) $@

$(BUILD)/$(TARGET).bin: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O binary $< $@

$(BUILD)/$(TARGET).hex: $(BUILD)/$(TARGET).elf
	$(OBJCOPY) -O ihex $< $@

flash: $(BUILD)/$(TARGET).elf
	STM32_Programmer_CLI -c port=SWD -w $< -v -rst

clean:
	rm -rf $(BUILD)

-include $(APP_OBJ:.o=.d) $(HAL_OBJ:.o=.d)

.PHONY: all clean flash
