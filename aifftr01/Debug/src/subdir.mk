################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
LD_SRCS += \
../src/lscript.ld 

C_SRCS += \
../src/algotst.c \
../src/bmark_lite.c \
../src/crc.c \
../src/heap.c \
../src/platform.c \
../src/thal.c \
../src/thlib.c 

OBJS += \
./src/algotst.o \
./src/bmark_lite.o \
./src/crc.o \
./src/heap.o \
./src/platform.o \
./src/thal.o \
./src/thlib.o 

C_DEPS += \
./src/algotst.d \
./src/bmark_lite.d \
./src/crc.d \
./src/heap.d \
./src/platform.d \
./src/thal.d \
./src/thlib.d 


# Each subdirectory must supply rules for building sources it contributes
src/%.o: ../src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: ARM R5 gcc compiler'
	armr5-none-eabi-gcc -DARMR5 -Wall -O0 -g3 -c -fmessage-length=0 -MT"$@" -mcpu=cortex-r5 -mfloat-abi=hard  -mfpu=vfpv3-d16 -ffunction-sections -I../../benchmark_bsp/psu_cortexr5_0/include -MMD -MP -MF"$(@:%.o=%.d)" -MT"$(@)" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


