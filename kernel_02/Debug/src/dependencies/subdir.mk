################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../src/dependencies/crc.c \
../src/dependencies/heap.c \
../src/dependencies/thal.c \
../src/dependencies/thlib.c 

OBJS += \
./src/dependencies/crc.o \
./src/dependencies/heap.o \
./src/dependencies/thal.o \
./src/dependencies/thlib.o 

C_DEPS += \
./src/dependencies/crc.d \
./src/dependencies/heap.d \
./src/dependencies/thal.d \
./src/dependencies/thlib.d 


# Each subdirectory must supply rules for building sources it contributes
src/dependencies/%.o: ../src/dependencies/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM R5 gcc compiler'
	armr5-none-eabi-gcc -DARMR5 -Wall -O0 -g3 -I"C:\Users\harvi\FYP\kernel_02\src\dependencies" -c -fmessage-length=0 -MT"$@" -mcpu=cortex-r5 -mfloat-abi=hard  -mfpu=vfpv3-d16 -v -I../../kernel_02_bsp/psu_cortexr5_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


