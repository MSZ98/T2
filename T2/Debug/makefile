################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
T2.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
T2.list \

OBJCOPY_BIN += \
T2.bin \


# All Target
all: T2.elf secondary-outputs

# Tool invocations
T2.elf: $(OBJS) $(USER_OBJS) C:\ST\STM32CubeIDE_1.3.0\STM32CubeIDE\workspace\T2\STM32F030K6TX_FLASH.ld
	arm-none-eabi-gcc -o "T2.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m0 -T"C:\ST\STM32CubeIDE_1.3.0\STM32CubeIDE\workspace\T2\STM32F030K6TX_FLASH.ld" --specs=nosys.specs -Wl,-Map="T2.map" -Wl,--gc-sections -static --specs=nano.specs -mfloat-abi=soft -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

T2.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "T2.list"
	@echo 'Finished building: $@'
	@echo ' '

T2.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "T2.bin"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) *
	-@echo ' '

secondary-outputs: $(SIZE_OUTPUT) $(OBJDUMP_LIST) $(OBJCOPY_BIN)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
