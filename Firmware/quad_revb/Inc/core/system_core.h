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
extern xTimerHandle controlTimerHandle;


#define CONTROL_LOOP_PERIOD_MS 2

enum SystemCoreNotificationFlags_T {
	EVENT_CONTROLLER_UPDATE = 0
};

#define sNotifySystemCore(event) xTaskNotify(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits)
#define sNotifySystemCoreFromISR(event, pxHigherPriorityTaskWoken) xTaskNotifyFromISR(systemCoreTaskHandle, (1 << (31 - (int)event)), eSetBits, pxHigherPriorityTaskWoken)
#define vSystemTimeMs() (configTICK_RATE_HZ / 1000.0f * xTaskGetTickCount())

#endif /* CORE_SYSTEM_CORE_H_ */
