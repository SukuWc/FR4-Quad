/*
 * system_logger.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "FreeRTOS.h"
#include "bsp/logger.h"
#include "limits.h"
#include "core/system_logger.h"
#include "core/system_core.h"
#include "core/pid.h"

extern xSemaphoreHandle loggerLockHandle;

static uint8_t buffer[256] = {0};
extern int16_t ax, ay, az, rotx, roty, rotz;
extern volatile float roll, pitch;
extern volatile float pitchAcc, rollAcc;
extern volatile float pitchGyro, rollGyro;
extern uint16_t ppm_values[8];
extern int32_t pwm_1, pwm_2, pwm_3, pwm_4;
extern int16_t pi,ro;
extern PIDStruct rollPid, pitchPid;

extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim4;


void logger_sendAccelerometerMessage(){
	//uint8_t msgLength = snprintf(buffer, 256, "AX: %+10d AY: %+10d AZ: %+10d ROTX: %+10d ROTY: %+10d ROTZ: %+10d\r\n", ax, ay, az, rotx, roty, rotz);
	//uint8_t msgLength = snprintf(buffer, 256, "PITCH: %8.4f ROLL: %8.4f ACC_PITCH: %8.4f ACC_ROLL: %8.4f GYRO_PITCH: %8.4f GYRO_ROLL: %8.4f ACCX: %+5d ACCY: %+5d ACCZ: %+5d \r\n", pitch, roll, pitchAcc, rollAcc, pitchGyro, rollGyro, ax, ay, az);
	//uint8_t msgLength = snprintf(buffer, 256, "%5.5d %5.5d %5.5d %5.5d %5.5d %5.5d %5.5d %5.5d  %4.4d %4.4d %4.4d %4.4d  %4.4d %4.4d %4.4d %4.4d\r\n", ppm_values[0], ppm_values[1], ppm_values[2], ppm_values[3], ppm_values[4], ppm_values[5], ppm_values[6], ppm_values[7], pwm_1, pwm_2, pwm_3, pwm_4, htim3.Instance->CCR4, htim3.Instance->CCR2, htim2.Instance->CCR1, htim4.Instance->CCR4 );
	uint8_t msgLength = snprintf(buffer, 256,
			"%d,%d,%d,%d,%d,%d,%f,%f,%f,%f,%f,%f,%d,%d,%d,%d,%d,%d,%f,%f\r\n",
			ax, ay, az,
			rotx, roty, rotz,
			pitchAcc, rollAcc,
			pitchGyro, rollGyro,
			pitch, roll,
			pwm_1, pwm_2, pwm_3, pwm_4,
			pi,ro,
			pitchPid.integral_part, rollPid.integral_part);
	sendMessage(buffer, msgLength);
}

void SendLogEvent(const void* argument){
	sNotifyLogger(ACCELERATION_SENSOR_UPDATED);
}

void SystemLoggerTask(const void* argument){
	for(;;){}
	/*uint32_t notifiedValue;
	uint8_t leadingZeroIndex;
	xTimerStart(sendLogTimerHandle, 0);
	for(;;){
		xSemaphoreTake(loggerLockHandle, portMAX_DELAY);
		if (xTaskNotifyWait(0x00, ULONG_MAX, &notifiedValue, portMAX_DELAY)){
			while ((leadingZeroIndex = __CLZ(notifiedValue)) != 32){
				notifiedValue &= UINT32_MAX >> (leadingZeroIndex + 1);
				switch(leadingZeroIndex){
				case ACCELERATION_SENSOR_UPDATED: logger_sendAccelerometerMessage(); break;
				}
			}
		}
	}*/
}

void messageFinishedFromISR(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(loggerLockHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
