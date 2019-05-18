/*
 * system_position.h
 *
 *  Created on: 12 May 2019
 *      Author: danim
 */

#ifndef CORE_SYSTEM_POSITION_H_
#define CORE_SYSTEM_POSITION_H_

#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include "math.h"
#include "core/kalman/MadgwickAHRS.h"

enum SystemPositionNotificationFlags_T {
	EVENT_POSITION_UPDATE = 0
};
extern xTaskHandle systemCoreTaskHandle;

#define sNotifySystemPosition(event) xTaskNotify(systemPositionHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifySystemPositionFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)

//Protected by positionDataMutex
extern volatile float position_pitch;
extern volatile float position_roll;
extern volatile float position_yaw;
extern volatile float position_yawSpeed;
extern volatile float position_pitchSpeed;
extern volatile float position_rollSpeed;
extern volatile float position_height;

extern SemaphoreHandle_t positionDataMutexHandle;
extern TaskHandle_t systemPositionHandle;
#endif /* CORE_SYSTEM_POSITION_H_ */
