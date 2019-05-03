/*
 * system_core.h
 *
 *  Created on: 4 Mar 2019
 *      Author: danim
 */

#ifndef CORE_SYSTEM_CORE_H_
#define CORE_SYSTEM_CORE_H_

#include "FreeRTOS.h"
#include "task.h"

/// From main.c
extern xTaskHandle systemCoreTaskHandle;
#ifndef __SIMULATOR__
extern xTimerHandle controlTimerHandle;
extern xTimerHandle positionUpdateTimerHandle;
extern xTimerHandle sendLogTimerHandle;
#endif

#define CONTROL_LOOP_PERIOD_MS 10
#define POSITION_LOOP_PERIOD_MS 1
#define LOGGER_LOOP_PERIOD_MS 20

enum SystemCoreNotificationFlags_T {
	EVENT_CONTROLLER_UPDATE = 0,
	EVENT_POSITION_UPDATE
};

#define sNotifySystemCore(event) xTaskNotify(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifySystemCoreFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)
#define vSystemTimeMs() (configTICK_RATE_HZ / 1000.0f * xTaskGetTickCount())

#endif /* CORE_SYSTEM_CORE_H_ */
