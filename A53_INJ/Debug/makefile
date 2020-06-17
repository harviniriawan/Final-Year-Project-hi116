################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/subdir.mk
-include subdir.mk
-include objects.mk

ifneq ($(MAKECMDGOALS),clean)
ifneq ($(strip $(S_UPPER_DEPS)),)
-include $(S_UPPER_DEPS)
endif
ifneq ($(strip $(C_DEPS)),)
-include $(C_DEPS)
endif
endif

-include ../makefile.defs

# Add inputs and outputs from these tool invocations to the build variables 
ELFSIZE += \
A53_INJ.elf.size \


# All Target
all: A53_INJ.elf secondary-outputs

# Tool invocations
A53_INJ.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM v8 32 Bit gcc linker'
	arm-none-eabi-gcc -march=armv7-a -mfpu=vfpv3 -mfloat-abi=hard -Wl,-T -Wl,../src/lscript.ld -L../../A53_INJ_bsp/psu_cortexa53_0/lib -o "A53_INJ.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

A53_INJ.elf.size: A53_INJ.elf
	@echo 'Invoking: ARM v8 32 Bit Print Size'
	arm-none-eabi-size A53_INJ.elf  |tee "A53_INJ.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(S_UPPER_DEPS)$(C_DEPS)$(ELFSIZE) A53_INJ.elf
	-@echo ' '

secondary-outputs: $(ELFSIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
