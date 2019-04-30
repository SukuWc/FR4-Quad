/*
 * init.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "bsp/init.h"
#include "bsp/mpu9250.h"
#include "bsp/mpu9250_regs.h"
#include "main.h"
#include "device_interface.h"

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;
extern I2C_HandleTypeDef hi2c2;

static int16_t i2c_read(uint16_t address, uint8_t* buffer, uint16_t length){
	uint8_t addr = address;
	if (HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, &addr, 1, 10) == HAL_OK){
		if (HAL_I2C_Master_Receive(&hi2c2, 0x68 << 1, buffer, length, 10) == HAL_OK){
			return length;
		} else {
			return -1;
		}
	} else {
		return -1;
	}
}

static uint8_t i2c_buffer[16] = {0};
static int16_t i2c_write(uint16_t address, uint8_t* buffer, uint16_t length){
	i2c_buffer[0] = address;
	for (uint16_t i = 0; i < length; i++){
		i2c_buffer[i+1] = buffer[i];
	}
	if (HAL_I2C_Master_Transmit(&hi2c2, 0x68 << 1, i2c_buffer, length + 1, 10) == HAL_OK){
		return length;
	} else {
		return -1;
	}
}

static uint8_t spi_buffer[16] = {0};
static uint8_t rx_buffer[16] = {0};
static int16_t spi_read(uint16_t address, uint8_t* buffer, uint16_t length){
	spi_buffer[0] = address;
	spi_buffer[0] |= 0x80;
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	if (HAL_SPI_TransmitReceive(&hspi1, spi_buffer, rx_buffer, length + 1, 10) == HAL_OK){
		for (uint16_t i = 0; i < length; i++){
			buffer[i] = rx_buffer[i + 1];
		}
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		return length;
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		return -1;
	}
}
static int16_t spi_write(uint16_t address, uint8_t* buffer, uint16_t length){
	spi_buffer[0] = address;
	spi_buffer[0] &= 0x7f;
	for (uint16_t i = 0; i < length; i++){
		spi_buffer[i+1] = buffer[i];
	}
	HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_RESET);
	if (HAL_SPI_Transmit(&hspi1, spi_buffer, length + 1, 10) == HAL_OK){
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(1);
		return length;
	} else {
		HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);
		HAL_Delay(1);
		return -1;
	}
}


DeviceInterface mpu_i2c;
Mpu9250Device mpu_device;

void initBsp(){
	HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	htim2.Instance->CCR1 = 1023;
	htim3.Instance->CCR2 = 1023;
	htim3.Instance->CCR4 = 1023;
	htim4.Instance->CCR4 = 1023;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);

	mpu_i2c.read = i2c_read;
	mpu_i2c.write = i2c_write;

	mpu_device.interface = &mpu_i2c;

	//mpu9250_initialize(&mpu_device);
	volatile uint8_t status;
	while(mpu9250_getClockSource(&mpu_device) != MPU9250_CLOCK_PLL_XGYRO){
		MX_RESET_I2C();
		mpu9250_setClockSource(&mpu_device, MPU9250_CLOCK_PLL_XGYRO);
	}
	while(mpu9250_getSleepEnabled(&mpu_device) != 0){
		MX_RESET_I2C();
		mpu9250_setSleepEnabled(&mpu_device, 0);
	}
	while (mpu9250_getFullScaleAccelRange(&mpu_device) != 3){
		MX_RESET_I2C();
		mpu9250_setFullScaleAccelRange(&mpu_device, MPU9250_ACCEL_FS_16);
	}
	while (mpu9250_getFullScaleGyroRange(&mpu_device) != 3){
		MX_RESET_I2C();
		mpu9250_setFullScaleGyroRange(&mpu_device, MPU9250_GYRO_FS_2000);
	}
	while (mpu9250_getFChoice_b(&mpu_device) != 0){
		MX_RESET_I2C();
		mpu9250_setFChoice_b(&mpu_device, 0);
	}
	while (mpu9250_getDLPFMode(&mpu_device) != 3){
		MX_RESET_I2C();
		mpu9250_setDLPFMode(&mpu_device, 3);
	}
	while (mpu9250_getAccelDPFL(&mpu_device) != 3){
		MX_RESET_I2C();
		mpu9250_setAccelDPFL(&mpu_device, 3);
	}
	while (mpu9250_getAccelF_b(&mpu_device) != 0){
		MX_RESET_I2C();
		mpu9250_setAccelF_b(&mpu_device, 0);
	}

	while (mpu9250_getSlaveAddress(&mpu_device, 0) != 0x8c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x8c);
	}
	while (mpu9250_getSlaveDataLength(&mpu_device, 0) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveDataLength(&mpu_device, 0, 1);
	}
	while (mpu9250_getSlaveRegister(&mpu_device, 0) != 0){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0);
	}
	while (mpu9250_getSlaveWriteMode(&mpu_device, 0) != 0){
		MX_RESET_I2C();
		mpu9250_setSlaveWriteMode(&mpu_device, 0, 0);
	}
	while (mpu9250_getSlaveEnabled(&mpu_device, 0) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveEnabled(&mpu_device, 0, 1);
	}
	while (mpu9250_getMasterClockSpeed(&mpu_device) != 8){
		MX_RESET_I2C();
		mpu9250_setMasterClockSpeed(&mpu_device, 8);
	}
	while (mpu9250_getSlaveReadWriteTransitionEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setSlaveReadWriteTransitionEnabled(&mpu_device, 1);
	}
	while(mpu9250_getI2CBypassEnabled(&mpu_device) != 0){
		MX_RESET_I2C();
		mpu9250_setI2CBypassEnabled(&mpu_device, 0);
	}
	while(mpu9250_getWaitForExternalSensorEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setWaitForExternalSensorEnabled(&mpu_device, 1);
	}
	while (mpu9250_getI2CMasterModeEnabled(&mpu_device) != 1){
		MX_RESET_I2C();
		mpu9250_setI2CMasterModeEnabled(&mpu_device, 1);
	}
	for (uint8_t i = 0; i < 6; i++){
		MX_RESET_I2C();
		mpu9250_resetI2CMaster(&mpu_device);
	}

	while(mpu9250_getSlaveAddress(&mpu_device, 0) != 0x0c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x0c);
	}
	while(mpu9250_getSlaveRegister(&mpu_device, 0) != 0x0a){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0xa);
	}
	for (uint8_t i = 0; i < 12; i++){
		MX_RESET_I2C();
		mpu9250_setSlaveOutputByte(&mpu_device, 0, 0x06);
	}
	HAL_Delay(10);

	while(mpu9250_getSlaveAddress(&mpu_device, 0) != 0x8c){
		MX_RESET_I2C();
		mpu9250_setSlaveAddress(&mpu_device, 0, 0x8c);
	}
	while(mpu9250_getSlaveRegister(&mpu_device, 0) != 0x02){
		MX_RESET_I2C();
		mpu9250_setSlaveRegister(&mpu_device, 0, 0x02);
	}
	while(mpu9250_getSlaveDataLength(&mpu_device, 0) != 8){
		MX_RESET_I2C();
		mpu9250_setSlaveDataLength(&mpu_device, 0, 8);
	}
	while(1){
		volatile uint8_t readValues[8];
		for (uint8_t i = 0; i < 8; i++){
			readValues[i] = mpu9250_getExternalSensorByte(&mpu_device, i);
		}
		HAL_Delay(100);
	}
	/*while(1){
		status = mpu9250_testConnection(&mpu_device);
	}*/
}
