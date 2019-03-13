/*
 * init.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "bsp/init.h"
#include "mpu9250.h"
#include "mpu9250_regs.h"
#include "main.h"

extern TIM_HandleTypeDef htim1, htim2, htim3, htim4;

void MX_RESET_I2C(void);

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

	//mpu9250_initialize();
	//HAL_Delay(100);
	while(mpu9250_getClockSource() != MPU9250_CLOCK_PLL_XGYRO){
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
	while (mpu9250_getFChoice_b() != 1){
		MX_RESET_I2C();
		mpu9250_setFChoice_b(1);
	}
	while (mpu9250_getDLPFMode() != 0){
		MX_RESET_I2C();
		mpu9250_setDLPFMode(0);
	}
	while (mpu9250_getAccelDPFL() != 0){
		MX_RESET_I2C();
		mpu9250_setAccelDPFL(0);
	}
	while (mpu9250_getAccelF_b() != 1){
		MX_RESET_I2C();
		mpu9250_setAccelF_b(1);
	}
}
