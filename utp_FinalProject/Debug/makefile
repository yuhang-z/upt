################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include Startup/subdir.mk
-include Src/subdir.mk
-include Drivers/STM32L4xx_HAL_Driver/Src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_DEPS)),)
-include $(S_DEPS)
endif
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
EXECUTABLES += \
utp_FinalProject.elf \

SIZE_OUTPUT += \
default.size.stdout \

OBJDUMP_LIST += \
utp_FinalProject.list \

OBJCOPY_BIN += \
utp_FinalProject.bin \


# All Target
all: utp_FinalProject.elf secondary-outputs

# Tool invocations
utp_FinalProject.elf: $(OBJS) $(USER_OBJS) /Users/norinmura/Desktop/upt/utp_FinalProject/STM32L475VGTX_FLASH.ld
	arm-none-eabi-gcc -o "utp_FinalProject.elf" @"objects.list" $(USER_OBJS) $(LIBS) -mcpu=cortex-m4 -T"/Users/norinmura/Desktop/upt/utp_FinalProject/STM32L475VGTX_FLASH.ld" --specs=nosys.specs -Wl,-Map="utp_FinalProject.map" -Wl,--gc-sections -static -L"/Users/norinmura/Desktop/upt/utp_FinalProject/Lib" --specs=nano.specs -mfpu=fpv4-sp-d16 -mfloat-abi=hard -mthumb -Wl,--start-group -lc -lm -Wl,--end-group
	@echo 'Finished building target: $@'
	@echo ' '

default.size.stdout: $(EXECUTABLES)
	arm-none-eabi-size  $(EXECUTABLES)
	@echo 'Finished building: $@'
	@echo ' '

utp_FinalProject.list: $(EXECUTABLES)
	arm-none-eabi-objdump -h -S $(EXECUTABLES) > "utp_FinalProject.list"
	@echo 'Finished building: $@'
	@echo ' '

utp_FinalProject.bin: $(EXECUTABLES)
	arm-none-eabi-objcopy  -O binary $(EXECUTABLES) "utp_FinalProject.bin"
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
