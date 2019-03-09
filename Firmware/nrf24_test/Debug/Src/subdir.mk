################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/gpio.c \
../Src/main.c \
../Src/nrf24.c \
../Src/nrf24_hal.c \
../Src/spi.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c 

OBJS += \
./Src/gpio.o \
./Src/main.o \
./Src/nrf24.o \
./Src/nrf24_hal.o \
./Src/spi.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o 

C_DEPS += \
./Src/gpio.d \
./Src/main.d \
./Src/nrf24.d \
./Src/nrf24_hal.d \
./Src/spi.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/prog/FR4-Quad/Firmware/nrf24_test/Inc" -I"C:/prog/FR4-Quad/Firmware/nrf24_test/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/prog/FR4-Quad/Firmware/nrf24_test/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/prog/FR4-Quad/Firmware/nrf24_test/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/prog/FR4-Quad/Firmware/nrf24_test/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


