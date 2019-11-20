################################################################################
# Automatically-generated file. Do not edit!
################################################################################

-include ../makefile.init

RM := rm -rf

# All of the sources participating in the build are defined here
-include sources.mk
-include src/dependencies/subdir.mk
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
kernel_02.elf.size \


# All Target
all: kernel_02.elf secondary-outputs

# Tool invocations
kernel_02.elf: $(OBJS) ../src/lscript.ld $(USER_OBJS)
	@echo 'Building target: $@'
	@echo 'Invoking: ARM R5 gcc linker'
	armr5-none-eabi-gcc -L"C:\Users\harvi\FYP\kernel_02\src\dependencies" -L"C:\Xilinx\SDK\2019.1\gnu\armr5\nt\gcc-arm-none-eabi\arm-none-eabi\lib\thumb\v7+fp\hard" -mcpu=cortex-r5 -mfloat-abi=hard -mfpu=vfpv3-d16 --specs=rdimon.specs -Wl,-T -Wl,../src/lscript.ld -L../../kernel_02_bsp/psu_cortexr5_0/lib -o "kernel_02.elf" $(OBJS) $(USER_OBJS) $(LIBS)
	@echo 'Finished building target: $@'
	@echo ' '

kernel_02.elf.size: kernel_02.elf
	@echo 'Invoking: ARM R5 Print Size'
	armr5-none-eabi-size kernel_02.elf  |tee "kernel_02.elf.size"
	@echo 'Finished building: $@'
	@echo ' '

# Other Targets
clean:
	-$(RM) $(EXECUTABLES)$(OBJS)$(S_UPPER_DEPS)$(C_DEPS)$(ELFSIZE) kernel_02.elf
	-@echo ' '

secondary-outputs: $(ELFSIZE)

.PHONY: all clean dependents
.SECONDARY:

-include ../makefile.targets
