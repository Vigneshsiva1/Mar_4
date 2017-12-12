################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/Losing_1_Notes_8bit.c \
../Src/Losing_2_Notes_8bit.c \
../Src/Shoot_1_Notes_8bit.c \
../Src/Shoot_2_Notes_8bit.c \
../Src/Winner_Notes_8bit.c \
../Src/main.c \
../Src/stm32f4xx_hal_msp.c \
../Src/stm32f4xx_it.c \
../Src/system_stm32f4xx.c 

OBJS += \
./Src/Losing_1_Notes_8bit.o \
./Src/Losing_2_Notes_8bit.o \
./Src/Shoot_1_Notes_8bit.o \
./Src/Shoot_2_Notes_8bit.o \
./Src/Winner_Notes_8bit.o \
./Src/main.o \
./Src/stm32f4xx_hal_msp.o \
./Src/stm32f4xx_it.o \
./Src/system_stm32f4xx.o 

C_DEPS += \
./Src/Losing_1_Notes_8bit.d \
./Src/Losing_2_Notes_8bit.d \
./Src/Shoot_1_Notes_8bit.d \
./Src/Shoot_2_Notes_8bit.d \
./Src/Winner_Notes_8bit.d \
./Src/main.d \
./Src/stm32f4xx_hal_msp.d \
./Src/stm32f4xx_it.d \
./Src/system_stm32f4xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo %cd%
	arm-none-eabi-gcc -mcpu=cortex-m4 -mthumb -mfloat-abi=hard -mfpu=fpv4-sp-d16 '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F407xx -I"C:/Users/Vaibhav/Downloads/Mar_Final/Mar_2/Inc" -I"C:/Users/Vaibhav/Downloads/Mar_Final/Mar_2/Drivers/STM32F4xx_HAL_Driver/Inc" -I"C:/Users/Vaibhav/Downloads/Mar_Final/Mar_2/Drivers/STM32F4xx_HAL_Driver/Inc/Legacy" -I"C:/Users/Vaibhav/Downloads/Mar_Final/Mar_2/Drivers/CMSIS/Device/ST/STM32F4xx/Include" -I"C:/Users/Vaibhav/Downloads/Mar_Final/Mar_2/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


