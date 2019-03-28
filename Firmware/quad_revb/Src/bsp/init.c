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
#include "spi.h"
#include "i2c.h"


void MX_RESET_I2C(void);


static uint8_t i2c_buffer[16] = {0};
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
static int16_t i2c_write(uint16_t address, uint8_t* buffer, uint16_t length){
	i2c_buffer[0] = address;
	for (uint16_t i = 0; i < length; i++){
		i2c_buffer[i+1] = buffer[i];
	}
	if (HAL_I2C_Master_Transmit(&hi2c2, 0x68, i2c_buffer, length + 1, 10) == HAL_OK){
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

void initBsp(){
	//HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	/*htim2.Instance->CCR1 = 1023;
	htim3.Instance->CCR2 = 1023;
	htim3.Instance->CCR4 = 1023;
	htim4.Instance->CCR4 = 1023;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);*/

	DeviceInterface mpu_i2c;
	mpu_i2c.read = i2c_read;
	mpu_i2c.write = i2c_write;

	Mpu9250Device mpu_device;
	mpu_device.interface = &mpu_i2c;

	//HAL_GPIO_WritePin(GPIOA, GPIO_PIN_15, GPIO_PIN_SET);

	//mpu9250_switchSPIEnabled(&mpu_device, 1);
	//uint8_t sendData = 0x10;
	//HAL_Delay(100);
	//spi_write(0x6A, &sendData, 1);
	//HAL_Delay(1000);
	//mpu9250_initialize(&mpu_device);
	volatile uint8_t status;
	while(1){
		status = mpu9250_testConnection(&mpu_device);
		HAL_Delay(10);
	}

	//mpu9250_initialize();
	//HAL_Delay(100);
	/*while(mpu9250_getClockSource() != MPU9250_CLOCK_PLL_XGYRO){
		MX_RESET_I2C();
		mpu9250_setClockSource(MPU9250_CLOCK_PLL_XGYRO);
	}
	while(mpu9250_getSleepEnabled() != 0){
		MX_RESET_I2C();
		mpu9250_setSleepEnabled(0);
	}
	while (mpu9250_getFullScaleAccelRange() != 3){
		MX_RESET_I2C();
		mpu9250_setFullScaleAccelRange(MPU9250_ACCEL_FS_16);
	}
	while (mpu9250_getFullScaleGyroRange() != 3){
		MX_RESET_I2C();
		mpu9250_setFullScaleGyroRange(MPU9250_GYRO_FS_2000);
	}
	while (mpu9250_getFChoice_b() != 0){
		MX_RESET_I2C();
		mpu9250_setFChoice_b(0);
	}
	while (mpu9250_getDLPFMode() != 3){
		MX_RESET_I2C();
		mpu9250_setDLPFMode(3);
	}
	while (mpu9250_getAccelDPFL() != 3){
		MX_RESET_I2C();
		mpu9250_setAccelDPFL(3);
	}
	while (mpu9250_getAccelF_b() != 0){
		MX_RESET_I2C();
		mpu9250_setAccelF_b(0);
	}*/
}
