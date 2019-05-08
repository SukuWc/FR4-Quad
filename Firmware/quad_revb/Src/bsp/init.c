/*
 * init.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "bsp/init.h"
#include "bsp/mpu9250.h"
#include "bsp/mpu9250_regs.h"
#include "bsp/ibus_handler.h"
#include "main.h"
#include "tim.h"
#include "device_interface.h"
#include "communication_interface.h"
#include "bsp/devices.h"

void initBsp(){
	/*HAL_TIM_IC_Start_IT(&htim1, TIM_CHANNEL_2);

	htim2.Instance->CCR1 = 1023;
	htim3.Instance->CCR2 = 1023;
	htim3.Instance->CCR4 = 1023;
	htim4.Instance->CCR4 = 1023;

	HAL_TIM_PWM_Start(&htim2, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_4);*/

	initMotors();
	initIbusHandler();
	initDevices();

	//mpu9250_initialize(&mpu_device);

	/*while(1){
		volatile uint8_t readValues[8];
		for (uint8_t i = 0; i < 8; i++){
			readValues[i] = mpu9250_getExternalSensorByte(&mpu_device, i);
		}
		HAL_Delay(100);
	}*/
	/*while(1){
		status = mpu9250_testConnection(&mpu_device);
	}*/
}
