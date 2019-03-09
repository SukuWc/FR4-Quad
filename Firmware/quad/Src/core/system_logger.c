/*
 * system_logger.c
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */
#include "cmsis_os.h"
#include "bsp/logger.h"
#include "limits.h"
#include "core/system_logger.h"

extern osSemaphoreId loggerLockHandle;

static uint8_t buffer[256] = {0};
extern int16_t ax, ay, az, rotx, roty, rotz;
extern volatile float roll, pitch;

void logger_sendAccelerometerMessage(){
	//uint8_t msgLength = snprintf(buffer, 256, "AX: %+10d AY: %+10d AZ: %+10d ROTX: %+10d ROTY: %+10d ROTZ: %+10d\r\n", ax, ay, az, rotx, roty, rotz);
	uint8_t msgLength = snprintf(buffer, 256, "PITCH: %8.4f ROLL: %8.4f \r\n", pitch, roll);
	sendMessage(buffer, 0);
}

void SystemLoggerTask(const void* argument){
	uint32_t notifiedValue;
	uint8_t leadingZeroIndex;
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
	}
}

void messageFinishedFromISR(){
	BaseType_t xHigherPriorityTaskWoken = pdFALSE;
	xSemaphoreGiveFromISR(loggerLockHandle, &xHigherPriorityTaskWoken);
	portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}
