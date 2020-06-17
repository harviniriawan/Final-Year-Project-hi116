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
aifftr01.elf.size \


# All Target
all: aifftr01.elf secondary-outputs

# Tool invocations
aifftr01.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM R5 gcc linker'
	armr5-none-eabi-gcc -nodefaultlibs -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 --specs=nano.specs -ffunction-sections -Wl,-T -Wl,../src/lscript.ld -L../../benchmark_bsp/psu_cortexr5_0/lib -o "aifftr01.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

aifftr01.elf.size: aifftr01.elf
	@echo 'Invoking: ARM R5 Print Size'
	armr5-none-eabi-size aifftr01.elf  |tee "aifftr01.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(S_UPPER_DEPS)$(C_DEPS)$(ELFSIZE) aifftr01.elf
	-@echo ' '

secondary-outputs: $(ELFSIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
