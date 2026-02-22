##########################################################
# 30 kW PDU Firmware — STM32G474RET6
# ARM GCC Makefile
##########################################################

########## Build configuration ##########
TARGET = pdu-firmware
BUILD_DIR = build

# Toolchain
PREFIX = arm-none-eabi-
CC = $(PREFIX)gcc
AS = $(PREFIX)gcc -x assembler-with-cpp
CP = $(PREFIX)objcopy
SZ = $(PREFIX)size
HEX = $(CP) -O ihex
BIN = $(CP) -O binary -S

# MCU flags
CPU = -mcpu=cortex-m4
FPU = -mfpu=fpv4-sp-d16
FLOAT-ABI = -mfloat-abi=hard
MCU = $(CPU) -mthumb $(FPU) $(FLOAT-ABI)

# Macros
C_DEFS = \
-DUSE_HAL_DRIVER \
-DSTM32G474xx

# Include paths
C_INCLUDES = \
-ICore/Inc \
-IApp/StateMachine \
-IApp/Control \
-IApp/ADC \
-IApp/Protection \
-IApp/CAN \
-IApp/PowerSequence \
-IApp/Diagnostics \
-IDrivers/STM32G4xx_HAL_Driver/Inc \
-IDrivers/STM32G4xx_HAL_Driver/Inc/Legacy \
-IDrivers/CMSIS/Device/ST/STM32G4xx/Include \
-IDrivers/CMSIS/Include

########## Source files ##########
# Project sources (strict warnings)
C_SOURCES = \
$(wildcard Core/Src/*.c) \
$(wildcard App/StateMachine/*.c) \
$(wildcard App/Control/*.c) \
$(wildcard App/ADC/*.c) \
$(wildcard App/Protection/*.c) \
$(wildcard App/CAN/*.c) \
$(wildcard App/PowerSequence/*.c) \
$(wildcard App/Diagnostics/*.c)

# Vendor sources (relaxed warnings)
VENDOR_SOURCES = \
$(wildcard Drivers/STM32G4xx_HAL_Driver/Src/*.c)

# Combined
C_SOURCES += $(VENDOR_SOURCES)

ASM_SOURCES = \
Core/Src/startup_stm32g474retx.s

########## Compiler flags ##########
ASFLAGS = $(MCU) $(C_DEFS) -Wall -fdata-sections -ffunction-sections

CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -Wall -Wextra -Wshadow \
-fdata-sections -ffunction-sections -std=c11

# Debug build by default; override with `make RELEASE=1`
ifdef RELEASE
CFLAGS += -O2 -DNDEBUG
else
CFLAGS += -Og -g3 -gdwarf-2
endif

# Generate dependency files
CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

########## Linker flags ##########
LDSCRIPT = STM32G474RETx_FLASH.ld

LDFLAGS = $(MCU) -specs=nano.specs -T$(LDSCRIPT) \
-lc -lm -lnosys \
-Wl,-Map=$(BUILD_DIR)/$(TARGET).map,--cref \
-Wl,--gc-sections \
-Wl,--print-memory-usage

########## Build rules ##########
OBJECTS = $(addprefix $(BUILD_DIR)/,$(notdir $(C_SOURCES:.c=.o)))
vpath %.c $(sort $(dir $(C_SOURCES)))

OBJECTS += $(addprefix $(BUILD_DIR)/,$(notdir $(ASM_SOURCES:.s=.o)))
vpath %.s $(sort $(dir $(ASM_SOURCES)))

all: $(BUILD_DIR)/$(TARGET).elf $(BUILD_DIR)/$(TARGET).hex $(BUILD_DIR)/$(TARGET).bin
	@$(SZ) $<

# Vendor sources — suppress unused-parameter and other warnings from ST HAL
VENDOR_CFLAGS = $(MCU) $(C_DEFS) $(C_INCLUDES) -Wall \
-fdata-sections -ffunction-sections -std=c11 \
-Wno-unused-parameter -Wno-shadow
ifdef RELEASE
VENDOR_CFLAGS += -O2 -DNDEBUG
else
VENDOR_CFLAGS += -Og -g3 -gdwarf-2
endif
VENDOR_CFLAGS += -MMD -MP -MF"$(@:%.o=%.d)"

# Vendor object list for pattern matching
VENDOR_OBJS = $(addprefix $(BUILD_DIR)/,$(notdir $(VENDOR_SOURCES:.c=.o)))

$(BUILD_DIR)/%.o: %.c Makefile | $(BUILD_DIR)
	$(if $(filter $@,$(VENDOR_OBJS)),\
	$(CC) -c $(VENDOR_CFLAGS) $< -o $@,\
	$(CC) -c $(CFLAGS) $< -o $@)

$(BUILD_DIR)/%.o: %.s Makefile | $(BUILD_DIR)
	$(AS) -c $(ASFLAGS) $< -o $@

$(BUILD_DIR)/$(TARGET).elf: $(OBJECTS) Makefile
	$(CC) $(OBJECTS) $(LDFLAGS) -o $@

$(BUILD_DIR)/%.hex: $(BUILD_DIR)/%.elf
	$(HEX) $< $@

$(BUILD_DIR)/%.bin: $(BUILD_DIR)/%.elf
	$(BIN) $< $@

$(BUILD_DIR):
	mkdir -p $@

########## Flash ##########
flash: $(BUILD_DIR)/$(TARGET).hex
	st-flash --format ihex write $<

########## Static analysis ##########
lint:
	cppcheck --enable=all --suppress=missingIncludeSystem \
		--std=c11 --platform=arm32-wchar_t2 \
		$(C_INCLUDES) $(C_DEFS) \
		--addon=misra \
		--error-exitcode=1 \
		Core/Src App/

########## Clean ##########
clean:
	rm -rf $(BUILD_DIR)

########## Dependencies ##########
-include $(wildcard $(BUILD_DIR)/*.d)

.PHONY: all clean flash lint
