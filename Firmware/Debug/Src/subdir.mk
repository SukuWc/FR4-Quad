################################################################################
# Automatically-generated file. Do not edit!
################################################################################

# Add inputs and outputs from these tool invocations to the build variables 
C_SRCS += \
../Src/adc.c \
../Src/bmp280.c \
../Src/freertos.c \
../Src/gpio.c \
../Src/i2c.c \
../Src/i2cif.c \
../Src/main.c \
../Src/mpu9250.c \
../Src/spi.c \
../Src/stm32f1xx_hal_msp.c \
../Src/stm32f1xx_it.c \
../Src/system_stm32f1xx.c \
../Src/tim.c \
../Src/usart.c \
../Src/usb.c 

OBJS += \
./Src/adc.o \
./Src/bmp280.o \
./Src/freertos.o \
./Src/gpio.o \
./Src/i2c.o \
./Src/i2cif.o \
./Src/main.o \
./Src/mpu9250.o \
./Src/spi.o \
./Src/stm32f1xx_hal_msp.o \
./Src/stm32f1xx_it.o \
./Src/system_stm32f1xx.o \
./Src/tim.o \
./Src/usart.o \
./Src/usb.o 

C_DEPS += \
./Src/adc.d \
./Src/bmp280.d \
./Src/freertos.d \
./Src/gpio.d \
./Src/i2c.d \
./Src/i2cif.d \
./Src/main.d \
./Src/mpu9250.d \
./Src/spi.d \
./Src/stm32f1xx_hal_msp.d \
./Src/stm32f1xx_it.d \
./Src/system_stm32f1xx.d \
./Src/tim.d \
./Src/usart.d \
./Src/usb.d 


# Each subdirectory must supply rules for building sources it contributes
Src/%.o: ../Src/%.c
	@echo 'Building file: $<'
	@echo 'Invoking: MCU GCC Compiler'
	@echo $(PWD)
	arm-none-eabi-gcc -mcpu=cortex-m3 -mthumb -mfloat-abi=soft '-D__weak=__attribute__((weak))' '-D__packed=__attribute__((__packed__))' -DUSE_HAL_DRIVER -DSTM32F103xB -I"C:/prog/FR4-Quad/Firmware/Inc" -I"C:/prog/FR4-Quad/Firmware/Drivers/STM32F1xx_HAL_Driver/Inc" -I"C:/prog/FR4-Quad/Firmware/Drivers/STM32F1xx_HAL_Driver/Inc/Legacy" -I"C:/prog/FR4-Quad/Firmware/Middlewares/Third_Party/FreeRTOS/Source/include" -I"C:/prog/FR4-Quad/Firmware/Middlewares/Third_Party/FreeRTOS/Source/CMSIS_RTOS" -I"C:/prog/FR4-Quad/Firmware/Middlewares/Third_Party/FreeRTOS/Source/portable/GCC/ARM_CM3" -I"C:/prog/FR4-Quad/Firmware/Drivers/CMSIS/Device/ST/STM32F1xx/Include" -I"C:/prog/FR4-Quad/Firmware/Drivers/CMSIS/Include"  -Og -g3 -Wall -fmessage-length=0 -ffunction-sections -c -fmessage-length=0 -MMD -MP -MF"$(@:%.o=%.d)" -MT"$@" -o "$@" "$<"
	@echo 'Finished building: $<'
	@echo ' '


